"""
link_clamp_jaw.py — ShelfBot clamp_jaw_link (rail-grip jaw / joint housing)

Geometry: cylindrical orange housing with a rectangular slot cutout (jaw) that
          hooks over the shelf beam, plus an inboard flange plate for
          attachment to clamp_arm_link.

Origin: centre of cylindrical housing axis = clamp_jaw_joint axis in URDF
Axis convention: jaw opening faces -Z (down onto rail); housing axis along Y

Output:
  stl/shelfbot_clamp_jaw_link.stl
  obj/shelfbot_clamp_jaw_link.obj
  mass_props/shelfbot_clamp_jaw_link.txt
"""

import trimesh
import trimesh.boolean
import trimesh.creation
import numpy as np
import json
import os

# ── Dimension constants (mm) ─────────────────────────────────────────────────
JAW_OD      = 60.0    # outer diameter of cylindrical housing
JAW_L       = 44.0    # length of cylindrical housing (along Y)
JAW_WALL_T  = 6.0     # wall thickness of housing (used for inner bore)

BEAM_W      = 40.0    # shelf beam width  (X)
BEAM_H      = 40.0    # shelf beam height (Z)
JAW_CLEAR   = 3.0     # clearance around beam

FLANGE_W    = 50.0    # flange plate width  (X)
FLANGE_H    = 30.0    # flange plate height (Z, sticks out from housing)
FLANGE_T    = 8.0     # flange plate thickness (Y)

DENSITY     = 2700e-9  # Aluminium

# ── Output paths ─────────────────────────────────────────────────────────────
BASE     = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STL_OUT  = os.path.join(BASE, "stl",       "shelfbot_clamp_jaw_link.stl")
OBJ_OUT  = os.path.join(BASE, "obj",       "shelfbot_clamp_jaw_link.obj")
PROP_OUT = os.path.join(BASE, "mass_props","shelfbot_clamp_jaw_link.txt")

# ── Cylindrical housing (axis along Y) ───────────────────────────────────────
housing = trimesh.creation.cylinder(radius=JAW_OD / 2, height=JAW_L, sections=48)
# Default cylinder axis is Z; rotate to Y
R = trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0])
housing.apply_transform(R)

# Hollow out interior (optional for mass accuracy — hollow tube)
inner_bore = trimesh.creation.cylinder(
    radius=JAW_OD / 2 - JAW_WALL_T, height=JAW_L + 2, sections=48
)
inner_bore.apply_transform(R)
housing_hollow = trimesh.boolean.difference([housing, inner_bore], engine='manifold')

# ── Jaw slot (rectangular cutout opening downward -Z) ────────────────────────
slot_w = BEAM_W + 2 * JAW_CLEAR
slot_h = BEAM_H + 2 * JAW_CLEAR
# Cutter: a tall box descending from below the housing centre
# We need to cut a channel that opens at the bottom of the cylinder
# Height of cutter must reach through the full radius
cutter_h = JAW_OD / 2 + slot_h
slot_cutter = trimesh.creation.box([slot_w, JAW_L + 2, cutter_h])
# Translate so the top of the cutter is at z=0 (housing centre), opening downward
slot_cutter.apply_translation([0, 0, -(cutter_h / 2)])

jaw = trimesh.boolean.difference([housing_hollow, slot_cutter], engine='manifold')

# ── Inboard flange plate (on the -Y face, toward the arm) ────────────────────
flange = trimesh.creation.box([FLANGE_W, FLANGE_T, FLANGE_H + JAW_OD / 2])
# Centre flange plate: align with housing centre, place at -Y face
flange.apply_translation([0, -(JAW_L / 2 + FLANGE_T / 2), (FLANGE_H - JAW_OD / 2) / 2])

final_jaw = trimesh.boolean.union([jaw, flange], engine='manifold')

# ── Mass properties ───────────────────────────────────────────────────────────
final_jaw.density = DENSITY
vol_mm3 = final_jaw.volume
mass_kg = vol_mm3 * DENSITY
com_mm  = final_jaw.center_mass
inertia = final_jaw.moment_inertia

print("=== clamp_jaw_link mass properties ===")
print(f"  Volume      : {vol_mm3:.1f} mm³")
print(f"  Mass        : {mass_kg*1000:.1f} g  ({mass_kg:.4f} kg)")
print(f"  Centre of mass (mm): x={com_mm[0]:.2f}  y={com_mm[1]:.2f}  z={com_mm[2]:.2f}")
print(f"  Inertia tensor (kg·mm²):")
for row in inertia:
    print(f"    {row[0]:12.4f}  {row[1]:12.4f}  {row[2]:12.4f}")
print(f"  Watertight  : {final_jaw.is_watertight}")
print(f"  Bounding box: {final_jaw.bounding_box.extents} mm")

# ── Export ─────────────────────────────────────────────────────────────────────
final_jaw.export(STL_OUT)
final_jaw.export(OBJ_OUT)

props = {
    "link": "clamp_jaw_link",
    "volume_mm3": float(vol_mm3),
    "mass_kg": float(mass_kg),
    "center_of_mass_mm": com_mm.tolist(),
    "inertia_tensor_kg_mm2": inertia.tolist(),
    "watertight": bool(final_jaw.is_watertight),
    "bounding_box_mm": final_jaw.bounding_box.extents.tolist(),
    "urdf_note": (
        "origin at cylinder housing centre; housing axis along Y. "
        "Jaw slot opens -Z. Flange extends in -Y direction toward clamp_arm. "
        "Beam slot sized for 40x40 mm shelf beam + 3 mm clearance."
    )
}
with open(PROP_OUT, "w") as f:
    json.dump(props, f, indent=2)

print(f"\nExported: {STL_OUT}")
print(f"Exported: {OBJ_OUT}")
print(f"Props:    {PROP_OUT}")
