"""
link_gripper.py — ShelfBot gripper_link (rope-covered distal jaw)

Geometry: central palm block with wrist bore + two curved C-section fingers
          that wrap around a product bottle. Visual version includes shallow
          grooves suggesting rope wrapping. Collision version is clean.

Origin: centre of palm wrist bore = gripper_joint axis
Axis convention: fingers open in ±Y; gripping direction along -X

Output:
  stl/shelfbot_gripper_link_visual.stl      (with rope grooves)
  stl/shelfbot_gripper_link_collision.stl   (clean, for sim collision)
  obj/shelfbot_gripper_link.obj
  mass_props/shelfbot_gripper_link.txt
"""

import trimesh
import trimesh.boolean
import trimesh.creation
import numpy as np
import json
import os
from shapely.geometry import Polygon as SPoly

# ── Dimension constants (mm) ─────────────────────────────────────────────────
PALM_W       = 65.0    # palm block width  (Y)
PALM_D       = 35.0    # palm block depth  (X, along gripping direction)
PALM_H       = 32.0    # palm block height (Z)

WRIST_R      = 26.0    # wrist bore radius (matches manip_proximal wrist flange)
WRIST_BORE_D = 10.0    # depth of wrist bore into palm (along +X)

FINGER_L     = 65.0    # finger length (along X, forward from palm)
FINGER_W     = 18.0    # finger width  (Z, thickness of finger)
INNER_R      = 36.0    # inner jaw radius (product contact surface)
OUTER_R      = INNER_R + FINGER_W  # outer jaw radius

GROOVE_DEPTH = 1.2     # groove depth for rope texture
GROOVE_W     = 3.0     # groove width
GROOVE_PITCH = 9.0     # groove spacing (centre to centre)

DENSITY      = 1200e-9  # Nylon / gripper material

# ── Output paths ─────────────────────────────────────────────────────────────
BASE      = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STL_VIS   = os.path.join(BASE, "stl",       "shelfbot_gripper_link_visual.stl")
STL_COL   = os.path.join(BASE, "stl",       "shelfbot_gripper_link_collision.stl")
OBJ_OUT   = os.path.join(BASE, "obj",       "shelfbot_gripper_link.obj")
PROP_OUT  = os.path.join(BASE, "mass_props","shelfbot_gripper_link.txt")

# ── Palm block with wrist bore ────────────────────────────────────────────────
palm = trimesh.creation.box([PALM_D, PALM_W, PALM_H])
# Wrist bore on +X face (where manip_proximal attaches)
wrist_bore = trimesh.creation.cylinder(radius=WRIST_R, height=WRIST_BORE_D + 1, sections=32)
R_bore = trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0])
wrist_bore.apply_transform(R_bore)
wrist_bore.apply_translation([PALM_D / 2, 0, 0])  # front face of palm
palm_with_bore = trimesh.boolean.difference([palm, wrist_bore], engine='manifold')

# ── Single finger (C-profile arc in Y-Z plane, extruded along X) ─────────────
# Arc spans from +45° to +135° (upper finger) — a C-shape opening inward (-Z)
def make_finger(side_sign: float) -> trimesh.Trimesh:
    """
    side_sign: +1 = top finger (opening -Z), -1 = bottom finger (opening +Z)
    The gripper closes by rotating both fingers about the Y axis.
    For the static open pose: top finger is at ~+Y half, bottom at ~-Y half,
    both curving inward in the Z direction.
    For simplicity, model each finger as a half-annulus (180° arc) in Y-Z,
    extruded along X (finger length).
    """
    # Build a 180° annular ring cross-section in Y-Z plane
    n_pts = 20
    # Outer arc: 180° from 0 to π (top half if side_sign>0)
    theta = np.linspace(0, np.pi, n_pts)
    if side_sign < 0:
        theta = theta + np.pi  # flip to bottom half

    outer_pts = np.column_stack([OUTER_R * np.cos(theta), OUTER_R * np.sin(theta)])
    inner_pts = np.column_stack([INNER_R * np.cos(theta[::-1]), INNER_R * np.sin(theta[::-1])])
    profile = np.vstack([outer_pts, inner_pts])
    poly = SPoly(profile)

    finger_mesh = trimesh.creation.extrude_polygon(poly, height=FINGER_L)
    # extrude_polygon goes along Z; rotate to go along X
    Rx = trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0])
    finger_mesh.apply_transform(Rx)
    # Translate finger to start at x = -PALM_D/2 (front face of palm = -X side)
    finger_mesh.apply_translation([-PALM_D / 2 - FINGER_L / 2, 0, 0])
    return finger_mesh

finger_top    = make_finger(+1)
finger_bottom = make_finger(-1)

# ── Merge palm + fingers ───────────────────────────────────────────────────────
gripper_clean = trimesh.boolean.union(
    [palm_with_bore, finger_top, finger_bottom], engine='manifold'
)

# ── Rope-groove texture (visual only) ────────────────────────────────────────
def add_grooves(base_mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    """Subtract a series of thin torus-like rings around each finger."""
    groove_cutters = []
    n_grooves = int(FINGER_L / GROOVE_PITCH)
    for i in range(n_grooves):
        x_pos = -PALM_D / 2 - GROOVE_PITCH * (i + 0.5)
        # Use a thin flat box as cutter (approximates a groove groove cut)
        # Cutter: a box that cuts across the finger arc at this X position
        cutter = trimesh.creation.box([GROOVE_W, (OUTER_R + 2) * 2, (OUTER_R + 2) * 2])
        cutter.apply_translation([x_pos, 0, 0])
        groove_cutters.append(cutter)
    if not groove_cutters:
        return base_mesh
    return trimesh.boolean.difference([base_mesh] + groove_cutters, engine='manifold')

gripper_visual = add_grooves(gripper_clean)

# ── Mass properties (use clean version) ──────────────────────────────────────
gripper_clean.density = DENSITY
vol_mm3 = gripper_clean.volume
mass_kg = vol_mm3 * DENSITY
com_mm  = gripper_clean.center_mass
inertia = gripper_clean.moment_inertia

print("=== gripper_link mass properties ===")
print(f"  Volume      : {vol_mm3:.1f} mm³")
print(f"  Mass        : {mass_kg*1000:.1f} g  ({mass_kg:.4f} kg)")
print(f"  Centre of mass (mm): x={com_mm[0]:.2f}  y={com_mm[1]:.2f}  z={com_mm[2]:.2f}")
print(f"  Inertia tensor (kg·mm²):")
for row in inertia:
    print(f"    {row[0]:12.4f}  {row[1]:12.4f}  {row[2]:12.4f}")
print(f"  Watertight (collision): {gripper_clean.is_watertight}")
print(f"  Watertight (visual)   : {gripper_visual.is_watertight}")
print(f"  Bounding box: {gripper_clean.bounding_box.extents} mm")

# ── Export ─────────────────────────────────────────────────────────────────────
gripper_visual.export(STL_VIS)
gripper_clean.export(STL_COL)
gripper_clean.export(OBJ_OUT)

props = {
    "link": "gripper_link",
    "volume_mm3": float(vol_mm3),
    "mass_kg": float(mass_kg),
    "center_of_mass_mm": com_mm.tolist(),
    "inertia_tensor_kg_mm2": inertia.tolist(),
    "watertight_collision": bool(gripper_clean.is_watertight),
    "watertight_visual": bool(gripper_visual.is_watertight),
    "bounding_box_mm": gripper_clean.bounding_box.extents.tolist(),
    "urdf_note": (
        "origin at palm wrist-bore centre (front face of palm). "
        "Fingers extend -X (forward toward product). "
        "Use collision STL for sim; visual STL for display. "
        "Inner jaw radius 36 mm fits product bottles 60-70 mm diameter."
    )
}
with open(PROP_OUT, "w") as f:
    json.dump(props, f, indent=2)

print(f"\nExported: {STL_VIS}")
print(f"Exported: {STL_COL}")
print(f"Exported: {OBJ_OUT}")
print(f"Props:    {PROP_OUT}")
