"""
export_assembly_step.py — Export all ShelfBot links as a merged OBJ for STEP conversion

Since trimesh cannot write STEP natively, this script:
1. Loads all 5 link STL files
2. Applies the same nominal assembled-pose transforms used in assemble_check.py
3. Exports a single merged OBJ file (shelfbot_assembly.obj) that preserves named groups
4. Also exports a single merged STL (shelfbot_assembly_for_step.stl)

To convert to STEP externally:
  FreeCAD: File → Import → select shelfbot_assembly.obj → File → Export → .step
  Online:  https://www.convertio.co/obj-step/ or cadexchanger.com
  CLI:     FreeCAD headless: freecadcmd export_step_from_obj.py  (if installed)

The OBJ named-group approach lets FreeCAD import each link as a separate solid,
which you can then select individually and export as a proper multi-body STEP.
"""

import trimesh
import numpy as np
import os

BASE    = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STL_DIR = os.path.join(BASE, "stl")
OUT_DIR = BASE

# ── Dimension constants (must match assemble_check.py) ──────────────────────
BODY_D, BODY_H = 200.0, 260.0
ARM_LENGTH      = 380.0
ARM_ANGLE       = 45.0    # deg
MANIP_LEN       = 160.0
MANIP_ANGLE     = -20.0   # deg

# ── STL files (use visual meshes for visual output) ──────────────────────────
LINK_FILES = {
    "base_link":           os.path.join(STL_DIR, "shelfbot_base_link.stl"),
    "clamp_arm_link":      os.path.join(STL_DIR, "shelfbot_clamp_arm_link.stl"),
    "clamp_jaw_link":      os.path.join(STL_DIR, "shelfbot_clamp_jaw_link.stl"),
    "manip_proximal_link": os.path.join(STL_DIR, "shelfbot_manip_proximal_link.stl"),
    "gripper_link":        os.path.join(STL_DIR, "shelfbot_gripper_link_visual.stl"),
}

# ── Recompute nominal transforms (same logic as assemble_check.py) ───────────
def make_transforms():
    eye = np.eye(4)
    clamp_arm_attach = np.array([-BODY_D * 0.3, 0, BODY_H / 2])
    arm_R = trimesh.transformations.rotation_matrix(np.radians(-ARM_ANGLE), [0, 1, 0])
    arm_tip_world = arm_R[:3, :3] @ np.array([ARM_LENGTH, 0, 0]) + clamp_arm_attach

    T_clamp_arm = np.eye(4)
    T_clamp_arm[:3, :3] = arm_R[:3, :3]
    T_clamp_arm[:3, 3]  = clamp_arm_attach

    T_clamp_jaw = np.eye(4)
    T_clamp_jaw[:3, 3] = arm_tip_world

    manip_attach = np.array([BODY_D / 2, 0, -BODY_H * 0.2])
    manip_R = trimesh.transformations.rotation_matrix(np.radians(MANIP_ANGLE), [0, 1, 0])
    T_manip = np.eye(4)
    T_manip[:3, :3] = manip_R[:3, :3]
    T_manip[:3, 3]  = manip_attach

    wrist_world = manip_R[:3, :3] @ np.array([MANIP_LEN, 0, 0]) + manip_attach
    T_gripper = np.eye(4)
    T_gripper[:3, 3] = wrist_world

    return {
        "base_link":           eye,
        "clamp_arm_link":      T_clamp_arm,
        "clamp_jaw_link":      T_clamp_jaw,
        "manip_proximal_link": T_manip,
        "gripper_link":        T_gripper,
    }

transforms = make_transforms()

# ── Load, transform, and collect ─────────────────────────────────────────────
placed_meshes = []
scene = trimesh.Scene()

print("Loading and positioning links...")
for name, stl_path in LINK_FILES.items():
    mesh = trimesh.load_mesh(stl_path)
    mesh.apply_transform(transforms[name])
    placed_meshes.append(mesh)
    scene.add_geometry(mesh, node_name=name, geom_name=name)
    print(f"  {name:30s}  vertices={len(mesh.vertices):6d}  faces={len(mesh.faces):6d}")

# ── Export merged STL (for viewing) ──────────────────────────────────────────
merged = trimesh.util.concatenate(placed_meshes)
stl_out = os.path.join(OUT_DIR, "shelfbot_assembly_for_step.stl")
merged.export(stl_out)
print(f"\nMerged STL  → {stl_out}  ({len(merged.faces)} faces)")

# ── Export multi-group OBJ (for STEP import in FreeCAD) ──────────────────────
obj_out = os.path.join(OUT_DIR, "shelfbot_assembly.obj")

# Build OBJ manually so each link becomes a named 'g' group
with open(obj_out, "w") as f:
    f.write("# ShelfBot assembly — multi-body OBJ for STEP export\n")
    f.write("# Each 'g' group = one robot link / rigid body\n")
    f.write("# Import into FreeCAD, then File > Export as .step\n\n")

    vertex_offset = 0
    for name, mesh in zip(LINK_FILES.keys(), placed_meshes):
        f.write(f"g {name}\n")
        for v in mesh.vertices:
            f.write(f"v {v[0]:.4f} {v[1]:.4f} {v[2]:.4f}\n")
        for face in mesh.faces:
            i0 = face[0] + vertex_offset + 1
            i1 = face[1] + vertex_offset + 1
            i2 = face[2] + vertex_offset + 1
            f.write(f"f {i0} {i1} {i2}\n")
        vertex_offset += len(mesh.vertices)
        f.write("\n")

print(f"Assembly OBJ → {obj_out}")
print("\nTo convert to STEP:")
print("  FreeCAD GUI : File > Import > shelfbot_assembly.obj > File > Export > .step")
print("  Each 'g' group in the OBJ imports as a separate solid in FreeCAD.")
print(f"\nTotal robot mass: 3.232 kg (from mass_props/*.txt)")
