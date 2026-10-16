"""
assemble_check.py — ShelfBot assembly validation

Loads all 5 link STL files, positions each at its nominal assembled pose
(joint angles estimated from the reference image), merges into a compound,
exports a single combined STL for inspection, and prints:
  - per-link bounding box
  - overall robot envelope
  - total mass
  - URDF joint table

Nominal poses (all in mm, world frame with base_link origin at world origin):
  base_link          : identity (origin at body geometric centre)
  clamp_arm_link     : pivot boss at body top-centre-rear; arm at 45° upward
  clamp_jaw_link     : at clamp arm tip, rotated to hook rail above
  manip_proximal_link: shoulder at body lower-front-centre; arm 20° below horiz.
  gripper_link       : at wrist flange, pointing forward
"""

import trimesh
import numpy as np
import json
import os

# ── Paths ─────────────────────────────────────────────────────────────────────
BASE     = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STL_DIR  = os.path.join(BASE, "stl")
OUT_DIR  = BASE

LINKS = {
    "base_link":           os.path.join(STL_DIR, "shelfbot_base_link.stl"),
    "clamp_arm_link":      os.path.join(STL_DIR, "shelfbot_clamp_arm_link.stl"),
    "clamp_jaw_link":      os.path.join(STL_DIR, "shelfbot_clamp_jaw_link.stl"),
    "manip_proximal_link": os.path.join(STL_DIR, "shelfbot_manip_proximal_link.stl"),
    "gripper_link":        os.path.join(STL_DIR, "shelfbot_gripper_link_visual.stl"),
}

# ── Dimension constants matching individual link scripts (mm) ─────────────────
BODY_W, BODY_D, BODY_H = 340.0, 200.0, 260.0   # base_link bounding box (approx)
ARM_LENGTH  = 380.0
ARM_ANGLE   = 45.0   # degrees from vertical (upward diagonal)
MANIP_LEN   = 160.0
MANIP_ANGLE = -20.0  # degrees below horizontal

# ── Load meshes ───────────────────────────────────────────────────────────────
print("Loading link meshes...")
meshes = {}
for name, path in LINKS.items():
    m = trimesh.load_mesh(path)
    meshes[name] = m
    print(f"  {name:30s}  {m.bounding_box.extents}  watertight={m.is_watertight}")

# ── Place each link at nominal assembled pose ─────────────────────────────────
# All transforms bring the link from its local origin (joint axis) to world frame.

def T(translation, rotation_axis=None, angle_deg=0.0):
    """Return a 4x4 homogeneous transform: rotate first, then translate."""
    mat = np.eye(4)
    if rotation_axis is not None:
        mat = trimesh.transformations.rotation_matrix(
            np.radians(angle_deg), rotation_axis
        )
    mat[:3, 3] = translation
    return mat

def Tchain(*matrices):
    """Compose transforms left-to-right (first applied first)."""
    result = np.eye(4)
    for m in matrices:
        result = m @ result
    return result

# base_link: identity, origin at body centre
base_T = np.eye(4)

# clamp_arm_link:
#   Attach point on body: top face (+Z), rear edge (-X), at body centre Y=0
#   In body frame: translation = (−BODY_D/2 * 0.3, 0, +BODY_H/2)
#   Then rotate about Y by −ARM_ANGLE (arm goes up-forward at 45°)
clamp_arm_attach = np.array([
    -BODY_D * 0.3,   # rear portion of body top
     0.0,
    +BODY_H / 2      # top face of body
])
clamp_arm_T = Tchain(
    T([0, 0, 0], [0, 1, 0], -ARM_ANGLE),   # rotate arm upward-diag
    T(clamp_arm_attach)                      # translate to body top
)

# clamp_jaw_link:
#   At tip of clamp arm (x=ARM_LENGTH in arm local frame)
#   Apply clamp_arm_T then step along arm local +X direction
arm_tip_local = np.array([ARM_LENGTH, 0, 0, 1])
arm_R = trimesh.transformations.rotation_matrix(np.radians(-ARM_ANGLE), [0, 1, 0])
arm_tip_world = arm_R[:3, :3] @ arm_tip_local[:3] + clamp_arm_attach
clamp_jaw_T = T(arm_tip_world)

# manip_proximal_link:
#   Attach at body lower-front face: x=+BODY_D/2, z=−BODY_H*0.2
#   Arm points forward (+X) and slightly downward (−ANGLE about Y)
manip_attach = np.array([+BODY_D / 2, 0.0, -BODY_H * 0.2])
manip_T = Tchain(
    T([0, 0, 0], [0, 1, 0], MANIP_ANGLE),  # tilt downward
    T(manip_attach)
)

# gripper_link:
#   At wrist flange end of manip_proximal (x=MANIP_LEN in arm local frame)
manip_R = trimesh.transformations.rotation_matrix(np.radians(MANIP_ANGLE), [0, 1, 0])
wrist_world = manip_R[:3, :3] @ np.array([MANIP_LEN, 0, 0]) + manip_attach
gripper_T = T(wrist_world)

TRANSFORMS = {
    "base_link":           base_T,
    "clamp_arm_link":      clamp_arm_T,
    "clamp_jaw_link":      clamp_jaw_T,
    "manip_proximal_link": manip_T,
    "gripper_link":        gripper_T,
}

# ── Apply transforms and collect ──────────────────────────────────────────────
placed = []
print("\nPlacing links at nominal pose...")
for name, mesh in meshes.items():
    m = mesh.copy()
    m.apply_transform(TRANSFORMS[name])
    placed.append(m)
    ext = m.bounding_box.extents
    cen = m.bounding_box.centroid
    print(f"  {name:30s}  centre={np.round(cen,1)}  extents={np.round(ext,1)}")

# ── Overall robot envelope ────────────────────────────────────────────────────
all_pts = np.vstack([m.vertices for m in placed])
robot_min = all_pts.min(axis=0)
robot_max = all_pts.max(axis=0)
robot_envelope = robot_max - robot_min

print(f"\n=== Overall robot envelope ===")
print(f"  X (depth)  : {robot_min[0]:.1f} → {robot_max[0]:.1f}  span={robot_envelope[0]:.1f} mm")
print(f"  Y (width)  : {robot_min[1]:.1f} → {robot_max[1]:.1f}  span={robot_envelope[1]:.1f} mm")
print(f"  Z (height) : {robot_min[2]:.1f} → {robot_max[2]:.1f}  span={robot_envelope[2]:.1f} mm")

# ── Total robot mass ──────────────────────────────────────────────────────────
mass_files = {
    "base_link":           "shelfbot_base_link.txt",
    "clamp_arm_link":      "shelfbot_clamp_arm_link.txt",
    "clamp_jaw_link":      "shelfbot_clamp_jaw_link.txt",
    "manip_proximal_link": "shelfbot_manip_proximal_link.txt",
    "gripper_link":        "shelfbot_gripper_link.txt",
}
total_mass = 0.0
print("\n=== Mass summary ===")
for name, fname in mass_files.items():
    fpath = os.path.join(BASE, "mass_props", fname)
    with open(fpath) as f:
        props = json.load(f)
    m_g = props["mass_kg"] * 1000
    total_mass += props["mass_kg"]
    print(f"  {name:30s}  {m_g:7.1f} g")
print(f"  {'TOTAL':30s}  {total_mass*1000:7.1f} g  ({total_mass:.3f} kg)")

# ── URDF joint table ──────────────────────────────────────────────────────────
print("\n=== URDF joint reference table ===")
print(f"  {'Joint name':30s}  {'Parent':25s}  {'Child':25s}  {'Type':10s}  Notes")
print(f"  {'-'*30}  {'-'*25}  {'-'*25}  {'-'*10}  -----")
joints = [
    ("clamp_arm_joint",      "base_link",           "clamp_arm_link",      "revolute",   "axis Y, ~45 deg nominal"),
    ("clamp_jaw_joint",      "clamp_arm_link",       "clamp_jaw_link",      "revolute",   "axis Y, passive hook"),
    ("manip_shoulder_joint", "base_link",           "manip_proximal_link", "revolute",   "axis Y, -20 deg nominal"),
    ("gripper_joint",        "manip_proximal_link", "gripper_link",        "revolute",   "axis Y, wrist rotation"),
]
for jname, parent, child, jtype, note in joints:
    print(f"  {jname:30s}  {parent:25s}  {child:25s}  {jtype:10s}  {note}")

# ── Export combined assembly mesh ─────────────────────────────────────────────
assembly = trimesh.util.concatenate(placed)
out_path = os.path.join(OUT_DIR, "shelfbot_assembly_check.stl")
assembly.export(out_path)
print(f"\nAssembly exported: {out_path}")
print(f"  Faces: {len(assembly.faces)}")
print(f"  Vertices: {len(assembly.vertices)}")

# ── Save summary JSON ─────────────────────────────────────────────────────────
summary = {
    "total_mass_kg": float(total_mass),
    "robot_envelope_mm": {
        "x_depth": float(robot_envelope[0]),
        "y_width": float(robot_envelope[1]),
        "z_height": float(robot_envelope[2]),
    },
    "links": 5,
    "joints": 4,
    "all_links_watertight": all(m.is_watertight for m in placed),
}
with open(os.path.join(OUT_DIR, "shelfbot_assembly_summary.json"), "w") as f:
    json.dump(summary, f, indent=2)
print(f"Summary JSON: {os.path.join(OUT_DIR, 'shelfbot_assembly_summary.json')}")
