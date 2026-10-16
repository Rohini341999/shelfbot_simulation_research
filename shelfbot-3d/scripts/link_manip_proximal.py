"""
link_manip_proximal.py — ShelfBot manip_proximal_link (front arm upper link)

Geometry: rectangular-section beam with cylindrical shoulder pivot boss at
          origin end and an octagonal wrist flange at the distal end.

Origin: centre of shoulder pivot boss = manip_shoulder_joint axis
Axis convention: arm extends +X from origin

Output:
  stl/shelfbot_manip_proximal_link.stl
  obj/shelfbot_manip_proximal_link.obj
  mass_props/shelfbot_manip_proximal_link.txt
"""

import trimesh
import trimesh.boolean
import trimesh.creation
import numpy as np
import json
import os

# ── Dimension constants (mm) ─────────────────────────────────────────────────
LINK_LENGTH   = 160.0   # beam length (boss centre to wrist flange face)
SECTION_W     = 25.0    # beam cross-section width  (Y)
SECTION_H     = 20.0    # beam cross-section height (Z)
FILLET_R      = 2.5     # chamfer approximation radius

BOSS_R        = 10.0    # shoulder boss cylinder radius
BOSS_L        = 22.0    # shoulder boss length (along Y)

# Octagonal wrist flange: circumscribed radius
WRIST_R       = 26.0    # circumscribed radius of octagon (≈ 52 mm across flats)
WRIST_T       = 8.0     # flange thickness (along X)

DENSITY       = 2700e-9  # Aluminium

# ── Output paths ─────────────────────────────────────────────────────────────
BASE     = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STL_OUT  = os.path.join(BASE, "stl",       "shelfbot_manip_proximal_link.stl")
OBJ_OUT  = os.path.join(BASE, "obj",       "shelfbot_manip_proximal_link.obj")
PROP_OUT = os.path.join(BASE, "mass_props","shelfbot_manip_proximal_link.txt")

# ── Arm beam ──────────────────────────────────────────────────────────────────
beam = trimesh.creation.box([LINK_LENGTH, SECTION_W, SECTION_H])
beam.apply_translation([LINK_LENGTH / 2, 0, 0])

# Chamfer long edges (same helper approach as clamp_arm)
def chamfer_beam_edges(mesh, length, sw, sh, cr):
    from shapely.geometry import Polygon as SPoly
    cut_meshes = []
    hw, hh = sw / 2, sh / 2
    for (sy, sz) in [(1, 1), (1, -1), (-1, 1), (-1, -1)]:
        pts_yz = np.array([
            [sy * hw,          sz * hh         ],
            [sy * (hw - cr),   sz * hh         ],
            [sy * hw,          sz * (hh - cr)  ],
        ])
        tri = SPoly(pts_yz)
        cutter = trimesh.creation.extrude_polygon(tri, height=length + 2)
        Rx = trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0])
        cutter.apply_transform(Rx)
        cutter.apply_translation([length / 2, 0, 0])
        cut_meshes.append(cutter)
    return trimesh.boolean.difference([mesh] + cut_meshes, engine='manifold')

beam_c = chamfer_beam_edges(beam, LINK_LENGTH, SECTION_W, SECTION_H, FILLET_R)

# ── Shoulder pivot boss (cylinder along Y, at origin) ────────────────────────
boss = trimesh.creation.cylinder(radius=BOSS_R, height=BOSS_L, sections=32)
R_boss = trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0])
boss.apply_transform(R_boss)
# Sits at x=0, centred — this IS the origin

# ── Octagonal wrist flange at distal end (+X) ─────────────────────────────────
# Build an octagon in the Y-Z plane, extrude along X
n = 8
angles = np.linspace(0, 2 * np.pi, n, endpoint=False) + np.pi / n
oct_pts = np.column_stack([WRIST_R * np.cos(angles), WRIST_R * np.sin(angles)])

from shapely.geometry import Polygon as SPoly
oct_poly = SPoly(oct_pts)
flange = trimesh.creation.extrude_polygon(oct_poly, height=WRIST_T)
# Rotate extrusion axis from Z to X
R_f = trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0])
flange.apply_transform(R_f)
flange.apply_translation([LINK_LENGTH + WRIST_T / 2, 0, 0])

# ── Combine ───────────────────────────────────────────────────────────────────
arm = trimesh.boolean.union([beam_c, boss, flange], engine='manifold')

# ── Mass properties ───────────────────────────────────────────────────────────
arm.density = DENSITY
vol_mm3 = arm.volume
mass_kg = vol_mm3 * DENSITY
com_mm  = arm.center_mass
inertia = arm.moment_inertia

print("=== manip_proximal_link mass properties ===")
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
    "link": "manip_proximal_link",
    "volume_mm3": float(vol_mm3),
    "mass_kg": float(mass_kg),
    "center_of_mass_mm": com_mm.tolist(),
    "inertia_tensor_kg_mm2": inertia.tolist(),
    "watertight": bool(arm.is_watertight),
    "bounding_box_mm": arm.bounding_box.extents.tolist(),
    "urdf_note": (
        "origin at shoulder boss centre (x=0); arm extends +X. "
        "In URDF, parent joint places this at body lower-front face; "
        "shoulder joint axis is along Y (horizontal, parallel to shelf)."
    )
}
with open(PROP_OUT, "w") as f:
    json.dump(props, f, indent=2)

print(f"\nExported: {STL_OUT}")
print(f"Exported: {OBJ_OUT}")
print(f"Props:    {PROP_OUT}")
