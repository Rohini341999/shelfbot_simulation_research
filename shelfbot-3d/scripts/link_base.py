"""
link_base.py — ShelfBot base_link (central body / torso)

Geometry: faceted hexagonal-front box shell, hollow interior, rail-mount notches,
          camera-mount boss on front face.

Axis convention: x = forward (out of front face), y = left, z = up
Origin: geometric centre of outer bounding box

Output:
  stl/shelfbot_base_link.stl
  obj/shelfbot_base_link.obj
  mass_props/shelfbot_base_link.txt
"""

import trimesh
import trimesh.boolean
import trimesh.creation
import numpy as np
import json
import os

# ── Dimension constants (mm) ────────────────────────────────────────────────
BODY_W      = 340.0   # width  (Y axis, side to side)
BODY_D      = 200.0   # depth  (X axis, front to back)
BODY_H      = 260.0   # height (Z axis, bottom to top)
CHAMFER     = 40.0    # corner chamfer size on front-face polygon
SHELL_T     = 4.0     # wall thickness for hollow shell

NOTCH_W     = 30.0    # rail-mount notch width  (Z)
NOTCH_H     = 20.0    # rail-mount notch depth  (X)
NOTCH_D     = 40.0    # rail-mount notch depth into body (Y, from right face)
NOTCH_Z_OFF = 60.0    # notch centre offset from body centre (Z)

CAM_R       = 18.0    # camera boss cylinder radius
CAM_H       = 12.0    # camera boss protrusion from front face
CAM_Z_OFF   = 10.0    # camera boss Z offset from centre (slight upward)

DENSITY     = 1050e-9  # ABS plastic kg/mm³  (= 1050 kg/m³)

# ── Output paths ─────────────────────────────────────────────────────────────
BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STL_OUT  = os.path.join(BASE, "stl",       "shelfbot_base_link.stl")
OBJ_OUT  = os.path.join(BASE, "obj",       "shelfbot_base_link.obj")
PROP_OUT = os.path.join(BASE, "mass_props","shelfbot_base_link.txt")

# ── Build outer shell polygon (front-face hexagonal / chamfered box) ─────────
# We approximate the chamfered box as a convex polygon in the Y-Z plane,
# then extrude along X (depth).
# Corners of a box with 45° chamfers at the four vertical (Y-Z) edges:
hw = BODY_W / 2   # half-width
hh = BODY_H / 2   # half-height
c  = CHAMFER

# 8-vertex polygon (clockwise from top-left looking at front face):
#   top-left-chamfered, top-right-chamfered,
#   right-top-chamfered, right-bot-chamfered,
#   bot-right-chamfered, bot-left-chamfered,
#   left-bot-chamfered, left-top-chamfered
front_poly = np.array([
    [-hw + c,  hh      ],   # top-left → inset right
    [ hw - c,  hh      ],   # top-right → inset left
    [ hw,      hh - c  ],   # right-top → inset down
    [ hw,     -hh + c  ],   # right-bot → inset up
    [ hw - c, -hh      ],   # bot-right → inset left
    [-hw + c, -hh      ],   # bot-left  → inset right
    [-hw,     -hh + c  ],   # left-bot  → inset up
    [-hw,      hh - c  ],   # left-top  → inset down
], dtype=float)

# Extrude the polygon along X (depth)
outer = trimesh.creation.extrude_polygon(
    trimesh.path.polygons.Polygon(front_poly),
    height=BODY_D
)
# extrude_polygon extrudes along Z in trimesh — we'll rotate to align with X
# Rotate 90° around Y so the extrusion axis becomes X
R = trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0])
outer.apply_transform(R)
# Now the body extends from x=0 to x=BODY_D in the +X direction.
# Translate so origin is at body centre.
outer.apply_translation([-BODY_D / 2, 0, 0])

# ── Inner void (shell) ───────────────────────────────────────────────────────
# Build a slightly smaller version of the same polygon for the hollow interior
s = SHELL_T
inner_poly = np.array([
    [-hw + c + s,  hh  - s ],
    [ hw - c - s,  hh  - s ],
    [ hw - s,      hh - c - s ],
    [ hw - s,     -hh + c + s ],
    [ hw - c - s, -hh  + s ],
    [-hw + c + s, -hh  + s ],
    [-hw + s,     -hh + c + s ],
    [-hw + s,      hh - c - s ],
], dtype=float)

inner = trimesh.creation.extrude_polygon(
    trimesh.path.polygons.Polygon(inner_poly),
    height=BODY_D - 2 * SHELL_T
)
inner.apply_transform(R)
inner.apply_translation([-(BODY_D / 2 - SHELL_T), 0, 0])

body_shell = trimesh.boolean.difference([outer, inner], engine='manifold')

# ── Rail-mount notches (two symmetric slots on the right face, +Y side) ─────
# The robot clamps onto the rack upright on its right-hand side.
# Two notches, symmetric about Z=0, at ±NOTCH_Z_OFF
notch_meshes = []
for sign in [+1, -1]:
    notch = trimesh.creation.box([NOTCH_H, NOTCH_D + 1, NOTCH_W])
    # Position: flush with right face (+Y), centred on notch Z offset
    notch.apply_translation([0, hw - NOTCH_D / 2, sign * NOTCH_Z_OFF])
    notch_meshes.append(notch)

body_with_notches = trimesh.boolean.difference(
    [body_shell] + notch_meshes, engine='manifold'
)

# ── Camera mount boss (cylinder on front face, +X direction) ─────────────────
cam_boss = trimesh.creation.cylinder(radius=CAM_R, height=CAM_H, sections=32)
# Default cylinder axis is Z — rotate to align with X
Rcam = trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0])
cam_boss.apply_transform(Rcam)
# Place on front face centre (x = +BODY_D/2), slightly up
cam_boss.apply_translation([BODY_D / 2 + CAM_H / 2, 0, CAM_Z_OFF])

final_body = trimesh.boolean.union([body_with_notches, cam_boss], engine='manifold')

# ── Mass properties ──────────────────────────────────────────────────────────
final_body.density = DENSITY
vol_mm3   = final_body.volume
mass_kg   = vol_mm3 * DENSITY
com_mm    = final_body.center_mass
inertia   = final_body.moment_inertia  # kg·mm²

print("=== base_link mass properties ===")
print(f"  Volume      : {vol_mm3:.1f} mm³  ({vol_mm3 / 1e6:.4f} L)")
print(f"  Mass        : {mass_kg*1000:.1f} g  ({mass_kg:.4f} kg)")
print(f"  Centre of mass (mm): x={com_mm[0]:.2f}  y={com_mm[1]:.2f}  z={com_mm[2]:.2f}")
print(f"  Inertia tensor (kg·mm²):")
for row in inertia:
    print(f"    {row[0]:12.4f}  {row[1]:12.4f}  {row[2]:12.4f}")
print(f"  Watertight  : {final_body.is_watertight}")
print(f"  Bounding box: {final_body.bounding_box.extents} mm")

# ── Export ────────────────────────────────────────────────────────────────────
final_body.export(STL_OUT)
final_body.export(OBJ_OUT)

props = {
    "link": "base_link",
    "volume_mm3": float(vol_mm3),
    "mass_kg": float(mass_kg),
    "center_of_mass_mm": com_mm.tolist(),
    "inertia_tensor_kg_mm2": inertia.tolist(),
    "watertight": bool(final_body.is_watertight),
    "bounding_box_mm": final_body.bounding_box.extents.tolist(),
    "urdf_note": "origin at geometric centre; x=forward, y=left, z=up"
}
with open(PROP_OUT, "w") as f:
    json.dump(props, f, indent=2)

print(f"\nExported: {STL_OUT}")
print(f"Exported: {OBJ_OUT}")
print(f"Props:    {PROP_OUT}")
