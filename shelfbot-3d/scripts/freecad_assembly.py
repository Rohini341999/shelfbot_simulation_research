"""
freecad_assembly.py — ShelfBot complete robot assembly with electronics

Generates a STEP file that can be opened directly in FreeCAD and viewed as a
fully assembled robot with all structural links AND electronics components
placed inside the body.

Run with:
    flatpak run org.freecad.FreeCAD --console freecad_assembly.py

Output:
    shelfbot-3d/step/shelfbot_assembly.step   ← open this in FreeCAD
    shelfbot-3d/step/shelfbot_assembly.FCStd  ← native FreeCAD project file

COORDINATE SYSTEM (all units = mm, matches URDF convention):
    x = forward (out of robot front face)
    y = left
    z = up
    Origin = wheel axle centre (75 mm above ground plane)

JOINT CHAIN (from URDF, converted m → mm):
    base_link origin:         x=0,   y=0,   z=0    (= wheel axle centre)
    base_link body visual:    x=0,   y=0,   z=+130  (body geometric centre)
    clamp_arm_joint:          x=-60, y=0,   z=+130  rel. to base_link
    climb_joint (arm tip):    x=+388 along arm  rel. to clamp_arm
    clamp_jaw_joint:          x=0,   y=0,   z=+40   rel. to climb_carriage
    manip_shoulder_joint:     x=+100,y=0,   z=-52   rel. to base_link
    gripper_joint:            x=+168 along arm  rel. to manip_proximal
"""

import sys
import os
import math

# ─── FreeCAD import ───────────────────────────────────────────────────────────
try:
    import FreeCAD
    import Part
    import Import          # for STEP export
    import Mesh            # for STL import
    # MeshPart.meshToShape does not exist in FreeCAD 1.1.1.
    # Use Part.Shape.makeShapeFromMesh instead (see import_stl_as_part below).
except ImportError:
    print("ERROR: This script must be run inside FreeCAD.")
    print("  flatpak run org.freecad.FreeCAD --console freecad_assembly.py")
    sys.exit(1)

# ─── Paths ────────────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT  = os.path.dirname(SCRIPT_DIR)          # shelfbot-3d/
STL_DIR    = os.path.join(REPO_ROOT, "stl")
STEP_DIR   = os.path.join(REPO_ROOT, "step")
os.makedirs(STEP_DIR, exist_ok=True)

STEP_OUT  = os.path.join(STEP_DIR, "shelfbot_assembly.step")
FCSTD_OUT = os.path.join(STEP_DIR, "shelfbot_assembly.FCStd")

# ─── Dimensions (mm) — from URDF xacro properties converted m→mm ─────────────
BODY_W         = 340.0
BODY_D         = 200.0
BODY_H         = 260.0
WHEEL_R        = 75.0
WHEEL_W        = 40.0
WHEEL_SEP      = 300.0
CASTER_R       = 25.0

ARM_LENGTH     = 388.0
ARM_SECTION_W  = 30.0
ARM_SECTION_H  = 20.0

CARRIAGE_W     = 60.0
CARRIAGE_D     = 60.0
CARRIAGE_H     = 80.0

MANIP_LENGTH   = 168.0
MANIP_W        = 25.0
MANIP_H        = 20.0

PALM_W         = 65.0
PALM_D         = 35.0
PALM_H         = 32.0
FINGER_L       = 65.0
FINGER_W       = 18.0

CAMERA_MAST_H  = 200.0
CAMERA_MAST_R  = 15.0

# ─── Colors (R,G,B 0–1) ───────────────────────────────────────────────────────
C_AMBER   = (0.95, 0.55, 0.05)
C_BLACK   = (0.12, 0.12, 0.12)
C_ORANGE  = (0.90, 0.40, 0.02)
C_GREY    = (0.60, 0.60, 0.60)
C_RUBBER  = (0.15, 0.15, 0.15)
C_GREEN   = (0.10, 0.70, 0.20)   # electronics — PCBs
C_BLUE    = (0.20, 0.35, 0.80)   # electronics — battery
C_RED     = (0.80, 0.20, 0.20)   # electronics — motor drivers
C_YELLOW  = (0.95, 0.85, 0.10)   # electronics — sensors/IMU

# ─── Helper: create a colored solid part ──────────────────────────────────────

def _set_color(obj, color):
    """Apply color only when a GUI ViewObject is available (no-op in --console)."""
    try:
        obj.ViewObject.ShapeColor = color
        obj.ViewObject.Transparency = 0
    except Exception:
        pass   # ViewObject is None in headless / --console mode


def make_box(doc, name, w, d, h, pos, color):
    """Create a Part::Box feature at position (x,y,z) with (w=X, d=Y, h=Z)."""
    obj = doc.addObject("Part::Box", name)
    obj.Length = w
    obj.Width  = d
    obj.Height = h
    obj.Placement = FreeCAD.Placement(
        FreeCAD.Vector(pos[0], pos[1], pos[2]),
        FreeCAD.Rotation(FreeCAD.Vector(0, 0, 1), 0)
    )
    _set_color(obj, color)
    return obj


def make_cylinder(doc, name, radius, height, pos, axis, color):
    """Create a Part::Cylinder. axis: 'x'|'y'|'z'."""
    obj = doc.addObject("Part::Cylinder", name)
    obj.Radius = radius
    obj.Height = height
    rot_map = {
        'z': FreeCAD.Rotation(FreeCAD.Vector(0, 0, 1), 0),
        'y': FreeCAD.Rotation(FreeCAD.Vector(1, 0, 0), 90),
        'x': FreeCAD.Rotation(FreeCAD.Vector(0, 1, 0), -90),
    }
    obj.Placement = FreeCAD.Placement(
        FreeCAD.Vector(pos[0], pos[1], pos[2]),
        rot_map.get(axis, rot_map['z'])
    )
    _set_color(obj, color)
    return obj


def make_sphere(doc, name, radius, pos, color):
    obj = doc.addObject("Part::Sphere", name)
    obj.Radius = radius
    obj.Placement = FreeCAD.Placement(
        FreeCAD.Vector(pos[0], pos[1], pos[2]),
        FreeCAD.Rotation()
    )
    _set_color(obj, color)
    return obj


def label_obj(obj, label):
    """Set the displayed label shown in the model tree."""
    obj.Label = label


def import_stl_as_part(doc, name, stl_path, pos, color):
    """
    Import an STL file and convert the mesh to a Part::Feature shape.

    FreeCAD 1.1.1 API: Part.Shape().makeShapeFromMesh(topology, tolerance)
    This produces a Compound of triangular faces — good enough for STEP export
    and visual inspection.  The mesh feature is removed after conversion.

    Returns the Part::Feature object, or None if the file is missing.
    """
    if not os.path.exists(stl_path):
        print(f"  [WARN] STL not found: {stl_path}  — using parametric fallback")
        return None

    # Count existing mesh objects before import so we can identify the new one
    before = set(o.Name for o in doc.Objects)
    Mesh.insert(stl_path, doc.Name)
    after  = set(o.Name for o in doc.Objects)
    new_names = after - before
    if not new_names:
        print(f"  [WARN] Mesh import returned nothing for {stl_path}")
        return None

    mesh_obj = doc.getObject(list(new_names)[0])

    # FreeCAD 1.1.1: convert via Part.Shape.makeShapeFromMesh
    shape = Part.Shape()
    shape.makeShapeFromMesh(mesh_obj.Mesh.Topology, 0.1)

    part_obj = doc.addObject("Part::Feature", name)
    part_obj.Shape = shape
    part_obj.Label = name
    part_obj.Placement = FreeCAD.Placement(
        FreeCAD.Vector(pos[0], pos[1], pos[2]),
        FreeCAD.Rotation()
    )
    _set_color(part_obj, color)

    # Remove the raw mesh feature (only keep the Part::Feature)
    doc.removeObject(mesh_obj.Name)
    return part_obj


# ═══════════════════════════════════════════════════════════════════════════════
# BUILD ASSEMBLY
# ═══════════════════════════════════════════════════════════════════════════════

print("=" * 60)
print("ShelfBot FreeCAD Assembly Generator")
print("=" * 60)

doc = FreeCAD.newDocument("ShelfBot_Assembly")

# ─────────────────────────────────────────────────────────────────────────────
# 1. BASE LINK (body shell)
#    URDF: base_link visual origin = (0, 0, +130) relative to wheel axle.
#    Assembly origin = wheel axle centre. So body bottom = z=0, top = z=260.
# ─────────────────────────────────────────────────────────────────────────────
print("\n[1] Body shell …")
stl_base = os.path.join(STL_DIR, "shelfbot_base_link.stl")

# Try to import the STL; if that fails, fall back to parametric box
base_obj = import_stl_as_part(
    doc, "Body_Shell", stl_base,
    pos=(-BODY_D/2, -BODY_W/2, 0),   # bottom-left-back corner so body centres at (0,0,130)
    color=C_AMBER
)
if base_obj is None:
    print("  → STL import failed, using parametric box")
    base_obj = make_box(doc, "Body_Shell",
                        w=BODY_D, d=BODY_W, h=BODY_H,
                        pos=(-BODY_D/2, -BODY_W/2, 0),
                        color=C_AMBER)
label_obj(base_obj, "01_Body_Shell")

# ─────────────────────────────────────────────────────────────────────────────
# 2. DRIVE WHEELS  (continuous joints about Y axis)
#    Wheel centres at z=0 (wheel axle), y = ±150 mm (wheel_sep/2)
# ─────────────────────────────────────────────────────────────────────────────
print("[2] Drive wheels …")
for side, y_sign in [("Left", +1), ("Right", -1)]:
    wh = make_cylinder(doc, f"Wheel_{side}",
                       radius=WHEEL_R,
                       height=WHEEL_W,
                       pos=(0, y_sign * WHEEL_SEP/2 - WHEEL_W/2, -WHEEL_R),
                       axis='y',
                       color=C_RUBBER)
    label_obj(wh, f"02_Wheel_{side}")

# ─────────────────────────────────────────────────────────────────────────────
# 3. REAR CASTER
#    Caster is at rear of base, z = wheel_radius - caster_radius above axle line
# ─────────────────────────────────────────────────────────────────────────────
print("[3] Rear caster …")
caster_x = -BODY_D/2 + 30.0       # from URDF: -body_d/2+0.03
caster_z = -(WHEEL_R - CASTER_R)  # from URDF: -(wheel_radius - caster_radius)
caster = make_sphere(doc, "Caster",
                     radius=CASTER_R,
                     pos=(caster_x, 0, caster_z),
                     color=C_GREY)
label_obj(caster, "03_Rear_Caster")

# ─────────────────────────────────────────────────────────────────────────────
# 4. CAMERA MAST (fixed post on top of body)
#    From URDF: origin xyz="0.040 0.0 body_h" rel. to base_link (= wheel axle)
#    → mast base at (40, 0, 260) in our assembly frame
# ─────────────────────────────────────────────────────────────────────────────
print("[4] Camera mast …")
mast_base_x = 40.0
mast_base_z = BODY_H   # 260 mm above wheel axle
mast = make_cylinder(doc, "Camera_Mast",
                     radius=CAMERA_MAST_R,
                     height=CAMERA_MAST_H,
                     pos=(mast_base_x, 0, mast_base_z),
                     axis='z',
                     color=C_BLACK)
label_obj(mast, "04_Camera_Mast")

# Camera pan servo block (on top of mast)
cam_pan_z = mast_base_z + CAMERA_MAST_H
cam_pan = make_box(doc, "Camera_Pan_Block",
                   w=40, d=40, h=20,
                   pos=(mast_base_x - 20, -20, cam_pan_z),
                   color=C_BLACK)
label_obj(cam_pan, "04b_Camera_Pan_Block")

# Camera body (slightly forward, 15° downward tilt approximated visually)
cam_body = make_box(doc, "Camera_Body",
                    w=60, d=80, h=35,
                    pos=(mast_base_x, -40, cam_pan_z + 20),
                    color=C_BLACK)
label_obj(cam_body, "04c_Camera_Body_RGBD")

# ─────────────────────────────────────────────────────────────────────────────
# 5. CLAMP ARM
#    Joint origin in URDF: xyz="-0.060 0.0 0.130" rel. to base_link (m→mm: -60, 0, 130)
#    Arm extends +X from joint origin (length 388 mm)
#    At rest (0 rad) the arm points forward (+X).
# ─────────────────────────────────────────────────────────────────────────────
print("[5] Clamp arm …")
clamp_arm_jx = -60.0
clamp_arm_jz = 130.0  # same as body_h/2 in world
stl_arm = os.path.join(STL_DIR, "shelfbot_clamp_arm_link.stl")
arm_obj = import_stl_as_part(
    doc, "Clamp_Arm",
    stl_arm,
    pos=(clamp_arm_jx, 0, clamp_arm_jz),
    color=C_BLACK
)
if arm_obj is None:
    arm_obj = make_box(doc, "Clamp_Arm",
                       w=ARM_LENGTH, d=ARM_SECTION_W, h=ARM_SECTION_H,
                       pos=(clamp_arm_jx, -ARM_SECTION_W/2, clamp_arm_jz - ARM_SECTION_H/2),
                       color=C_BLACK)
label_obj(arm_obj, "05_Clamp_Arm")

# ─────────────────────────────────────────────────────────────────────────────
# 6. CLIMB CARRIAGE (prismatic joint at tip of clamp arm)
#    Joint origin: +388 mm along arm from clamp_arm_joint, so in world frame:
#    x = clamp_arm_jx + 388, z = clamp_arm_jz  (at rest / 0 climb)
# ─────────────────────────────────────────────────────────────────────────────
print("[6] Climb carriage …")
carriage_x = clamp_arm_jx + ARM_LENGTH
carriage_z = clamp_arm_jz
carriage = make_box(doc, "Climb_Carriage",
                    w=CARRIAGE_W, d=CARRIAGE_D, h=CARRIAGE_H,
                    pos=(carriage_x - CARRIAGE_W/2,
                         -CARRIAGE_D/2,
                         carriage_z - CARRIAGE_H/2),
                    color=C_ORANGE)
label_obj(carriage, "06_Climb_Carriage")

# ─────────────────────────────────────────────────────────────────────────────
# 7. CLAMP JAW
#    Joint origin: +40 mm in Z from carriage centre
# ─────────────────────────────────────────────────────────────────────────────
print("[7] Clamp jaw …")
jaw_z = carriage_z + 40.0
stl_jaw = os.path.join(STL_DIR, "shelfbot_clamp_jaw_link.stl")
jaw_obj = import_stl_as_part(
    doc, "Clamp_Jaw",
    stl_jaw,
    pos=(carriage_x, 0, jaw_z),
    color=C_ORANGE
)
if jaw_obj is None:
    jaw_obj = make_cylinder(doc, "Clamp_Jaw",
                            radius=30, height=44,
                            pos=(carriage_x, 0, jaw_z),
                            axis='y',
                            color=C_ORANGE)
label_obj(jaw_obj, "07_Clamp_Jaw")

# ─────────────────────────────────────────────────────────────────────────────
# 8. MANIPULATOR ARM
#    Joint origin: xyz="0.100 0.0 -0.052" rel. to base_link (m→mm: +100, 0, -52)
#    Arm extends +X from joint (length 168 mm) at rest
# ─────────────────────────────────────────────────────────────────────────────
print("[8] Manipulator arm …")
manip_jx = 100.0
manip_jz = -52.0   # URDF z offset from wheel-axle origin
stl_manip = os.path.join(STL_DIR, "shelfbot_manip_proximal_link.stl")
manip_obj = import_stl_as_part(
    doc, "Manip_Arm",
    stl_manip,
    pos=(manip_jx, 0, manip_jz),
    color=C_BLACK
)
if manip_obj is None:
    manip_obj = make_box(doc, "Manip_Arm",
                         w=MANIP_LENGTH, d=MANIP_W, h=MANIP_H,
                         pos=(manip_jx, -MANIP_W/2, manip_jz - MANIP_H/2),
                         color=C_BLACK)
label_obj(manip_obj, "08_Manipulator_Arm")

# ─────────────────────────────────────────────────────────────────────────────
# 9. GRIPPER
#    Joint origin: +168 mm along manip arm from manip_shoulder_joint
# ─────────────────────────────────────────────────────────────────────────────
print("[9] Gripper …")
gripper_jx = manip_jx + MANIP_LENGTH
gripper_jz = manip_jz
stl_gripper = os.path.join(STL_DIR, "shelfbot_gripper_link_visual.stl")
gripper_obj = import_stl_as_part(
    doc, "Gripper",
    stl_gripper,
    pos=(gripper_jx, 0, gripper_jz),
    color=(0.45, 0.35, 0.25)
)
if gripper_obj is None:
    # Palm block
    palm = make_box(doc, "Gripper_Palm",
                    w=PALM_D, d=PALM_W, h=PALM_H,
                    pos=(gripper_jx, -PALM_W/2, gripper_jz - PALM_H/2),
                    color=(0.45, 0.35, 0.25))
    label_obj(palm, "09a_Gripper_Palm")
    # Left finger
    lfing = make_box(doc, "Gripper_Finger_L",
                     w=FINGER_L, d=FINGER_W, h=PALM_H,
                     pos=(gripper_jx, PALM_W/2, gripper_jz - PALM_H/2),
                     color=(0.45, 0.35, 0.25))
    label_obj(lfing, "09b_Gripper_Finger_Left")
    # Right finger
    rfing = make_box(doc, "Gripper_Finger_R",
                     w=FINGER_L, d=FINGER_W, h=PALM_H,
                     pos=(gripper_jx, -PALM_W/2 - FINGER_W, gripper_jz - PALM_H/2),
                     color=(0.45, 0.35, 0.25))
    label_obj(rfing, "09c_Gripper_Finger_Right")
    gripper_obj = palm

if gripper_obj is not None:
    label_obj(gripper_obj, "09_Gripper")

# ─────────────────────────────────────────────────────────────────────────────
# ELECTRONICS — placed inside the body cavity
#
# Body interior volume (with shell_t=4 mm walls):
#   x: -96 to +96 mm (BODY_D/2 - 4  = 96)
#   y: -166 to +166 mm (BODY_W/2 - 4 = 166)
#   z: 4 to 256 mm (4 mm floor clearance)
#
# Hardware prototype BOM:
#   - 2× Brushless DC motors + encoders  (drive wheels)
#   - 1× Raspberry Pi 4B                 (main computer)
#   - 1× Jetson Nano 4GB                 (perception / YOLO inference)
#   - 1× RoboClaw 2x60A motor driver     (drive)
#   - 1× Pololu Maestro 12-ch servo ctrl (arm/gripper servos)
#   - 1× 7-cell LiPo 26V 10Ah battery   (main power)
#   - 1× Intel RealSense D435i           (on mast — modelled above as Camera_Body)
#   - 1× Witmotion BWT901CL IMU          (inside body, top)
#   - 2× 24V Dynamixel servo             (clamp arm + manip shoulder)
#   - Power distribution PCB             (custom)
#   - Buck/boost converter board         (5V/12V rails)
# ─────────────────────────────────────────────────────────────────────────────
print("\n[E] Electronics …")

# ── Main battery (26V LiPo, largest component — floor of body) ────────────────
# Dimensions: ~190×110×60 mm (typical 10Ah LiPo 7S)
batt = make_box(doc, "Battery_LiPo_26V_10Ah",
                w=190, d=110, h=60,
                pos=(-95, -55, 4),   # centred in X/Y, 4 mm above floor
                color=C_BLUE)
label_obj(batt, "E01_Battery_26V_10Ah_LiPo")

# ── Raspberry Pi 4B (85×56×18 mm) ─────────────────────────────────────────────
rpi = make_box(doc, "Raspberry_Pi_4B",
               w=85, d=56, h=18,
               pos=(-42, -28, 68),   # on top of battery + 4 mm standoffs
               color=C_GREEN)
label_obj(rpi, "E02_RaspberryPi_4B")

# ── Jetson Nano 4GB (69×45×20 mm, Module+carrier) ────────────────────────────
jetson = make_box(doc, "Jetson_Nano_4GB",
                  w=69, d=45, h=20,
                  pos=(-34, 40, 68),  # next to RPi, left side
                  color=C_GREEN)
label_obj(jetson, "E03_Jetson_Nano_4GB")

# ── RoboClaw 2x60A motor driver (97×60×20 mm) ────────────────────────────────
roboclaw = make_box(doc, "RoboClaw_2x60A",
                    w=97, d=60, h=20,
                    pos=(-48, -28, 92),  # above RPi stack
                    color=C_RED)
label_obj(roboclaw, "E04_RoboClaw_2x60A_MotorDriver")

# ── Pololu Maestro 12-ch servo controller (38×28×10 mm) ──────────────────────
maestro = make_box(doc, "Pololu_Maestro_12ch",
                   w=38, d=28, h=10,
                   pos=(-19, -28, 116),
                   color=C_RED)
label_obj(maestro, "E05_Pololu_Maestro_12ch_ServoCtrl")

# ── Power distribution PCB (custom, 120×80×5 mm) ─────────────────────────────
pwr_pcb = make_box(doc, "Power_Distribution_PCB",
                   w=120, d=80, h=5,
                   pos=(-60, -40, 130),
                   color=C_GREEN)
label_obj(pwr_pcb, "E06_Power_Distribution_PCB")

# ── Buck/boost converter board (60×35×15 mm) ─────────────────────────────────
buck = make_box(doc, "Buck_Boost_Converter",
                w=60, d=35, h=15,
                pos=(-30, -28, 140),
                color=C_RED)
label_obj(buck, "E07_Buck_Boost_Converter_5V_12V")

# ── IMU (Witmotion BWT901CL, 51×36×9 mm) ─────────────────────────────────────
# Placed near body centre, high up for best measurement
imu = make_box(doc, "IMU_BWT901CL",
               w=51, d=36, h=9,
               pos=(-25, -18, 240),  # near top of body, centred
               color=C_YELLOW)
label_obj(imu, "E08_IMU_Witmotion_BWT901CL")

# ── Dynamixel servo #1 — clamp arm shoulder (52×40×40 mm)  ───────────────────
# Mounted on left interior wall, mid-height
dyn1 = make_box(doc, "Dynamixel_Servo_ClampArm",
                w=52, d=40, h=40,
                pos=(-96, -20, 110),   # left wall (x=-96)
                color=C_YELLOW)
label_obj(dyn1, "E09_Dynamixel_24V_ClampArm_Servo")

# ── Dynamixel servo #2 — manip shoulder (52×40×40 mm) ────────────────────────
dyn2 = make_box(doc, "Dynamixel_Servo_ManipArm",
                w=52, d=40, h=40,
                pos=(-96, -70, 110),   # left wall, further back
                color=C_YELLOW)
label_obj(dyn2, "E10_Dynamixel_24V_ManipShoulder_Servo")

# ─────────────────────────────────────────────────────────────────────────────
# Recompute & save
# ─────────────────────────────────────────────────────────────────────────────
print("\n[*] Recomputing model …")
doc.recompute()

print(f"[*] Saving FCStd → {FCSTD_OUT}")
doc.saveAs(FCSTD_OUT)

print(f"[*] Exporting STEP → {STEP_OUT}")
shapes_to_export = []
for obj in doc.Objects:
    if hasattr(obj, 'Shape') and not obj.Shape.isNull():
        shapes_to_export.append(obj)

if shapes_to_export:
    Import.export(shapes_to_export, STEP_OUT)
    print(f"    Exported {len(shapes_to_export)} shapes.")
else:
    print("    [WARN] No Part shapes found — STEP may be empty. Check STL imports.")

print("\n✓ Done.")
print(f"  Open in FreeCAD: {FCSTD_OUT}")
print(f"  Or import STEP:  {STEP_OUT}")
print()
print("  Labels in the model tree:")
print("  01_Body_Shell           — ABS torso (from STL or parametric box)")
print("  02_Wheel_Left/Right     — rubber drive wheels")
print("  03_Rear_Caster          — rear balance sphere")
print("  04_Camera_Mast*         — mast + pan block + RealSense D435i body")
print("  05_Clamp_Arm            — aluminium climbing arm")
print("  06_Climb_Carriage       — prismatic rail rider")
print("  07_Clamp_Jaw            — rail hook jaw")
print("  08_Manipulator_Arm      — distal reach arm")
print("  09_Gripper*             — palm + fingers")
print("  E01..E10                — electronics: battery, RPi, Jetson, drivers, IMU, servos")
