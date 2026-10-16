"""
link_clamp_arm.py — ShelfBot clamp_arm_link (upper diagonal arm)

Geometry: rectangular-section beam with cylindrical pivot boss at body end
          and flat interface pad at rail end. Filleted long edges.

Origin: centre of the pivot boss (= clamp_arm_joint axis in URDF)
Axis convention: x = along arm length (from body toward jaw), z = up when at 45°

Output:
  stl/shelfbot_clamp_arm_link.stl
  obj/shelfbot_clamp_arm_link.obj
  mass_props/shelfbot_clamp_arm_link.txt
"""

import trimesh
import trimesh.boolean
import trimesh.creation
import numpy as np
import json
import os

# ── Dimension constants (mm) ─────────────────────────────────────────────────
ARM_LENGTH   = 380.0   # total length of beam (boss centre to pad face)
SECTION_W    = 30.0    # beam cross-section width  (Y)
SECTION_H    = 20.0    # beam cross-section height (Z)
FILLET_R     = 3.0     # edge fillet radius (approximated by bevelled extrusion)

BOSS_R       = 12.0    # pivot boss cylinder radius
BOSS_L       = 24.0    # pivot boss cylinder length (sticks out ±12 mm in Y)

PAD_W        = 45.0    # interface pad width  (Y)
PAD_H        = 35.0    # interface pad height (Z)
PAD_T        = 8.0     # interface pad thickness (X, at arm tip)

DENSITY      = 2700e-9  # Aluminium 6061 kg/mm³

# ── Output paths ─────────────────────────────────────────────────────────────
BASE     = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STL_OUT  = os.path.join(BASE, "stl",       "shelfbot_clamp_arm_link.stl")
OBJ_OUT  = os.path.join(BASE, "obj",       "shelfbot_clamp_arm_link.obj")
PROP_OUT = os.path.join(BASE, "mass_props","shelfbot_clamp_arm_link.txt")

# ── Build arm beam ────────────────────────────────────────────────────────────
# Rectangular cross section in Y-Z, extruded along X from x=0 to x=ARM_LENGTH
beam = trimesh.creation.box([ARM_LENGTH, SECTION_W, SECTION_H])
# Default box is centred at origin — translate so pivot end is at x=0
beam.apply_translation([ARM_LENGTH / 2, 0, 0])

# ── Pivot boss (cylinder along Y axis at origin end) ─────────────────────────
boss = trimesh.creation.cylinder(radius=BOSS_R, height=BOSS_L, sections=32)
# Default cylinder axis is Z — rotate to Y axis
R_boss = trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0])
boss.apply_transform(R_boss)
# Boss is centred at origin → sits at the body-end of the arm (x=0)

# ── Interface pad at rail end ─────────────────────────────────────────────────
pad = trimesh.creation.box([PAD_T, PAD_W, PAD_H])
pad.apply_translation([ARM_LENGTH + PAD_T / 2, 0, 0])

# ── Fillet approximation: bevel the beam long edges ──────────────────────────
# Trimesh doesn't have native fillets; we chamfer by subtracting small triangular
# prisms from the four long edges (Y-min/max, Z-min/max corners along X).
def chamfer_beam(mesh, length, sw, sh, cr):
    """Remove a small 45° triangular prism from each of the 4 long beam edges."""
    cut_meshes = []
    hw, hh = sw / 2, sh / 2
    for (sy, sz) in [(1, 1), (1, -1), (-1, 1), (-1, -1)]:
        # Triangle in Y-Z plane: corner at (sy*hw, sz*hh),
        # inset by cr along each face
        pts_yz = np.array([
            [sy * hw,           sz * hh          ],
            [sy * (hw - cr),    sz * hh          ],
            [sy * hw,           sz * (hh - cr)   ],
        ])
        from shapely.geometry import Polygon as SPoly
        tri_poly = SPoly(pts_yz)
        cutter = trimesh.creation.extrude_polygon(tri_poly, height=length + 2)
        # Rotate to extrude along X
        Rx = trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0])
        cutter.apply_transform(Rx)
        cutter.apply_translation([length / 2, 0, 0])
        cut_meshes.append(cutter)
    return trimesh.boolean.difference([mesh] + cut_meshes, engine='manifold')

beam_chamfered = chamfer_beam(beam, ARM_LENGTH, SECTION_W, SECTION_H, FILLET_R)

# ── Combine all parts ─────────────────────────────────────────────────────────
arm = trimesh.boolean.union([beam_chamfered, boss, pad], engine='manifold')

# ── Mass properties ───────────────────────────────────────────────────────────
arm.density = DENSITY
vol_mm3 = arm.volume
mass_kg = vol_mm3 * DENSITY
com_mm  = arm.center_mass
inertia = arm.moment_inertia

print("=== clamp_arm_link mass properties ===")
print(f"  Volume      : {vol_mm3:.1f} mm³")
print(f"  Mass        : {mass_kg*1000:.1f} g  ({mass_kg:.4f} kg)")
print(f"  Centre of mass (mm): x={com_mm[0]:.2f}  y={com_mm[1]:.2f}  z={com_mm[2]:.2f}")
print(f"  Inertia tensor (kg·mm²):")
for row in inertia:
    print(f"    {row[0]:12.4f}  {row[1]:12.4f}  {row[2]:12.4f}")
print(f"  Watertight  : {arm.is_watertight}")
print(f"  Bounding box: {arm.bounding_box.extents} mm")

# ── Export ─────────────────────────────────────────────────────────────────────
arm.export(STL_OUT)
arm.export(OBJ_OUT)

props = {
    "link": "clamp_arm_link",
    "volume_mm3": float(vol_mm3),
    "mass_kg": float(mass_kg),
    "center_of_mass_mm": com_mm.tolist(),
    "inertia_tensor_kg_mm2": inertia.tolist(),
    "watertight": bool(arm.is_watertight),
    "bounding_box_mm": arm.bounding_box.extents.tolist(),
    "urdf_note": (
        "origin at boss centre (x=0); arm extends +X to x=ARM_LENGTH. "
        "In URDF, parent joint places this at body top-rear; "
        "rotate ~45 deg about Y to achieve diagonal pose."
    )
}
with open(PROP_OUT, "w") as f:
    json.dump(props, f, indent=2)

print(f"\nExported: {STL_OUT}")
print(f"Exported: {OBJ_OUT}")
print(f"Props:    {PROP_OUT}")
