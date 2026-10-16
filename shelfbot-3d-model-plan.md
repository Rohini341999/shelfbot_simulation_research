# ShelfBot 3D Model Plan — FreeCAD Python Scripting

## Overview

Create a clean, engineering-grade 3D model of the ShelfBot robot visible in the reference image, broken into **5 separate rigid-body link files** (one STEP + one STL per link). The model is built using **FreeCAD's Python scripting API** so every dimension, shape, and Boolean operation is reproducible and version-controllable as a `.py` script. No manual GUI work — every link is a script.

**Toolchain**: FreeCAD 1.x (freecadcmd for headless execution), Python scripting via `FreeCAD.newDocument()` / `Part` workbench.

**Output per link**: `shelfbot_<linkname>.FCStd` (native), `shelfbot_<linkname>.step` (STEP 214), `shelfbot_<linkname>.stl` (for visual/collision use).

**Source of truth for dimensions**: The reference image + shampoo bottle height used as a scale reference (~250 mm tall bottle). All dimensions are estimates to be refined once a physical measurement or CAD reference is available — the scripts use named constants at the top of each file so re-scaling is a single-line change.

---

## Kinematic Tree (link names)

```
base_link               ← central body / torso (the yellow hexagonal shell)
  └── clamp_arm_link    ← upper diagonal arm that grips the shelf rail
        └── clamp_jaw_link  ← the rotational joint housing / jaw at the rail end
  └── manip_proximal_link  ← front manipulator upper link
        └── gripper_link    ← rope-covered distal gripper jaw
```

Five links total. Joint axes and origins are defined in each script so they are ready for URDF `<joint>` definitions later.

---

## Sub-Task 1 — `base_link`: Central Body Shell

**Status**: [x] done — `stl/shelfbot_base_link.stl`, `obj/shelfbot_base_link.obj` produced. Mass: 1682.6 g, watertight.

### Intent
Build the main body — the compact, faceted amber torso that mounts on the shelf upright. This is the parent link to which all other links attach, so its joint-origin reference frame (geometric centre of the body, x-forward, y-left, z-up) must be established here.

### Expected Outcomes
- `shelfbot_base_link.FCStd`, `shelfbot_base_link.step`, `shelfbot_base_link.stl` exist.
- The body is a chamfered/faceted box (~340 × 200 × 260 mm, WxDxH) that approximates the hexagonal-front profile.
- A flat rail-mounting feature (two symmetric notch cutouts on the left face) represents the shelf beam interface zone.
- Mass properties are computable from FreeCAD (density = 1050 kg/m³ ABS plastic, shell thickness 4 mm) — the script prints mass, CoM, and inertia tensor to stdout.
- Origin is placed at the geometric centre of the body bounding box.

### Todo List
1. Create `scripts/link_base.py` with named dimension constants at the top (`BODY_W`, `BODY_D`, `BODY_H`, `CHAMFER_SIZE`, `SHELL_T`).
2. Build the outer shell: extrude a polygon sketch (6-sided front profile) along the depth axis.
3. Apply chamfers to the four vertical edges to match the faceted look in the image.
4. Shell the solid (hollow interior, 4 mm wall) using `Part.Shell` or a subtracted inner box.
5. Add two symmetric rectangular notches on the left lateral face (~30 × 20 mm) as the rail-grip interface zone.
6. Add a small boss/cylinder on the front face centre for the camera mount point.
7. Set material density, compute and print `mass`, `CenterOfMass`, `MatrixOfInertia`.
8. Export `.step` and `.stl` from the script using `Part.export()`.
9. Verify the file loads cleanly in FreeCAD GUI (no broken references).

### Relevant Context
- Reference image: the body label reads "BQUID" — the front face is slightly wider than deep, the top is rounded/chamfered.
- Scale reference: the two shampoo pump bottles on the shelf are ~250 mm tall; the body appears roughly equal in height.
- FreeCAD API: `import FreeCAD, Part; doc = FreeCAD.newDocument(); Part.show(shape); doc.saveAs(path)`.

---

## Sub-Task 2 — `clamp_arm_link`: Upper Diagonal Clamp Arm

**Status**: [x] done — `stl/shelfbot_clamp_arm_link.stl` produced. Mass: 665.3 g, watertight.

### Intent
Model the single rigid arm that extends upward-diagonally from the top of the body to the shelf rail. This is the structural member that keeps the robot clamped to the rack. Getting the length and angle right is critical for simulation stability later.

### Expected Outcomes
- `shelfbot_clamp_arm_link.FCStd`, `.step`, `.stl` exist.
- The arm is a rectangular-section beam (~30 × 20 mm cross-section, ~380 mm long) angled at approximately 45° from vertical in the image plane.
- A pivot boss (cylinder, 20 mm diameter, 20 mm long) is modelled at the body-end — this is joint `clamp_arm_joint` origin.
- The far end has a flat pad/plate (40 × 30 mm) as the interface face to `clamp_jaw_link`.
- Mass properties printed to stdout.

### Todo List
1. Create `scripts/link_clamp_arm.py` with constants (`ARM_LENGTH`, `SECTION_W`, `SECTION_H`, `ANGLE_DEG`).
2. Build the arm beam: extrude a rectangle of `SECTION_W × SECTION_H` along `ARM_LENGTH`.
3. Add a cylindrical pivot boss at the origin end (body attachment).
4. Add a flat rectangular interface pad at the rail end.
5. Apply 2 mm fillets to all long edges (the arm in the image looks rounded, not sharp).
6. Set origin at the centre of the pivot boss (= URDF joint axis location).
7. Compute and print mass properties.
8. Export `.step` and `.stl`.

### Relevant Context
- The arm appears roughly equal in length to the body height (~260 mm) — estimate `ARM_LENGTH = 380 mm` to account for the diagonal reach to the rail above.
- The orange joint housing visible at the rail end belongs to `clamp_jaw_link`, not this link.

---

## Sub-Task 3 — `clamp_jaw_link`: Rail-Grip Jaw / Joint Housing

**Status**: [x] done — `stl/shelfbot_clamp_jaw_link.stl` produced. Mass: 145.8 g, watertight.

### Intent
Model the orange rotational joint housing and jaw at the tip of the clamp arm. This is the part that physically hooks onto the horizontal shelf beam. It is a separate link because it may rotate (passive or actuated) relative to the arm when the robot traverses.

### Expected Outcomes
- `shelfbot_clamp_jaw_link.FCStd`, `.step`, `.stl` exist.
- Shape: a cylindrical housing body (~60 mm diameter, 40 mm long) with a curved jaw cutout that wraps around a beam of ~40 × 40 mm (standard Dexion/Boltless shelf upright section).
- Two curved jaw faces (inner radius = 25 mm) open downward to hook over the beam.
- A flat flange on the inboard face is the attachment face to `clamp_arm_link`.
- Origin at the centre of the cylindrical housing = `clamp_jaw_joint` axis.

### Todo List
1. Create `scripts/link_clamp_jaw.py` with constants (`JAW_OD`, `JAW_L`, `BEAM_W`, `BEAM_H`, `JAW_GAP`).
2. Build the cylindrical housing by extruding a circle.
3. Subtract a rectangular slot (beam cross-section + clearance) from the bottom of the cylinder to create the jaw channel.
4. Add the flange plate on the inboard face.
5. Apply 3 mm fillets to all exposed edges (the housing in the image is smooth and rounded).
6. Set origin at housing centre axis.
7. Compute and print mass properties.
8. Export `.step` and `.stl`.

### Relevant Context
- Standard Boltless/Dexion shelf beam is ~40 × 40 mm or ~50 × 30 mm — use 40 × 40 mm as the default, expose it as a constant.
- The jaw must be wide enough to hook over the beam but close enough to prevent lateral slop — this will matter in simulation contact.

---

## Sub-Task 4 — `manip_proximal_link`: Front Manipulator Upper Link

**Status**: [x] done — `stl/shelfbot_manip_proximal_link.stl` produced. Mass: 266.6 g, watertight.

### Intent
Model the first (proximal) link of the front manipulator arm — the rigid structural segment that extends forward from the lower-front face of the body. This is a black structural link, shorter than the clamp arm.

### Expected Outcomes
- `shelfbot_manip_proximal_link.FCStd`, `.step`, `.stl` exist.
- Shape: a rectangular-section beam (~25 × 20 mm cross-section, ~160 mm long) with a pivot boss at the body end and a wrist flange at the gripper end.
- The pivot boss (shoulder joint) is the origin.
- The wrist flange is a flat octagonal plate (~50 mm across) — the interface face to `gripper_link`.
- Mass properties printed.

### Todo List
1. Create `scripts/link_manip_proximal.py` with constants (`LINK_LENGTH`, `SECTION_W`, `SECTION_H`, `WRIST_FLANGE_D`).
2. Build the beam by extruding a rectangle.
3. Add a cylindrical shoulder pivot boss at the origin end.
4. Add an octagonal wrist flange at the distal end (extrude an octagonal sketch, 8 mm thick).
5. Apply 2 mm fillets to long edges.
6. Set origin at centre of shoulder pivot boss.
7. Compute and print mass properties.
8. Export `.step` and `.stl`.

### Relevant Context
- The proximal link in the image appears roughly 60% the length of the clamp arm — estimate ~160 mm.
- The arm appears to angle slightly downward from horizontal when reaching the shelf level, so the shoulder joint axis is likely horizontal (parallel to the shelf).

---

## Sub-Task 5 — `gripper_link`: Rope-Covered Distal Gripper

**Status**: [x] done — `stl/shelfbot_gripper_link_visual.stl` + `stl/shelfbot_gripper_link_collision.stl` produced. Mass: 472.3 g. Collision mesh watertight.

### Intent
Model the gripper — the rope/textile-wrapped distal jaw that contacts the product. This is a compliant-looking gripper; in simulation it will be treated as a rigid body with high-friction contact properties. The rope texture is cosmetic (not structural), so the mesh should be the underlying rigid form, not a detailed rope surface.

### Expected Outcomes
- `shelfbot_gripper_link.FCStd`, `.step`, `.stl` exist.
- Shape: two curved jaw fingers (~80 mm span when open, ~20 mm finger width, ~60 mm finger length) attached to a central palm block (~60 × 40 × 30 mm).
- The palm block has a cylindrical bore matching the wrist flange diameter of `manip_proximal_link`.
- A simplified rope-texture representation: 3 mm rounded ridges around each finger (no actual rope geometry — just the visual impression of wrapping, done as a simple helix-like extrusion or omitted for the collision mesh).
- Finger inner radius = ~30 mm (bottle body diameter ~30–35 mm).
- Origin at centre of palm bore = `gripper_joint` axis.

### Todo List
1. Create `scripts/link_gripper.py` with constants (`FINGER_SPAN`, `FINGER_W`, `FINGER_L`, `PALM_W`, `PALM_D`, `PALM_H`, `INNER_R`).
2. Build the palm block as a box with a cylindrical bore on the wrist-face.
3. Build one finger as: extrude a curved C-profile (arc + straight sides) along `FINGER_L`.
4. Mirror the finger about the palm centre plane to get the second finger.
5. Fuse palm + both fingers into one solid.
6. Optional visual: add 3 shallow grooves (~1 mm deep, 3 mm wide, spaced 8 mm) wrapping the fingers to suggest rope wrapping — skip for the collision STL.
7. Set origin at centre of the palm bore.
8. Compute and print mass properties.
9. Export two STLs: `shelfbot_gripper_link_visual.stl` (with grooves) and `shelfbot_gripper_link_collision.stl` (without grooves, simpler geometry).
10. Export `.step` (full visual version).

### Relevant Context
- The gripper in the image appears to be holding a pump-bottle (~70 mm body diameter) — size the jaw inner radius accordingly.
- Rope texture is AI-generated artistic detail; do not attempt to model real rope. Grooves are sufficient.

---

## Sub-Task 6 — Assembly Check Script

**Status**: [x] done — `scripts/assemble_check.py` runs cleanly. `shelfbot_assembly_check.stl` + `shelfbot_assembly_summary.json` produced. Total robot mass: 3.23 kg. Envelope: 370 x 260 x 599 mm.

### Intent
Write a single FreeCAD Python script that imports all 5 link STEP files, positions them at their nominal assembled pose (using approximate joint angles from the image), and exports a single assembly STEP and a rendered screenshot. This validates that all links fit together correctly before any URDF work begins.

### Expected Outcomes
- `scripts/assemble_check.py` exists and runs without errors via `freecadcmd`.
- The assembled STEP `shelfbot_assembly_check.step` opens in FreeCAD/CAD viewer and looks correct.
- A `shelfbot_assembly_check.png` top-front-iso screenshot is saved.
- No interpenetration between adjacent links at nominal pose (visually confirmed).

### Todo List
1. Create `scripts/assemble_check.py`.
2. Import each `shelfbot_<link>.step` using `Part.read()`.
3. Apply a `FreeCAD.Placement` to each link that positions it at its nominal assembled location (hardcode the nominal joint angles from the image, e.g. clamp_arm at 45° from body top, manipulator at 30° below horizontal).
4. Fuse all into a compound shape (compound, not union — keeps them separate for visual inspection).
5. Export the compound as `shelfbot_assembly_check.step`.
6. Use FreeCAD's offscreen rendering to save a screenshot if available; otherwise note that manual screenshot is needed.
7. Print a per-link bounding-box report so overall robot envelope is documented (expected: roughly 500 mm wide × 400 mm tall × 350 mm deep at nominal pose).

### Relevant Context
- This step is purely a sanity check — no constraints or motion simulation yet.
- Joint placement reference: in the image, the clamp arm rises from the top-rear of the body at ~45° and the manipulator extends forward-downward at ~20° below horizontal.

---

## File/Folder Layout

```
shelfbot-3d/
├── scripts/
│   ├── link_base.py
│   ├── link_clamp_arm.py
│   ├── link_clamp_jaw.py
│   ├── link_manip_proximal.py
│   ├── link_gripper.py
│   └── assemble_check.py
├── step/
│   ├── shelfbot_base_link.step
│   ├── shelfbot_clamp_arm_link.step
│   ├── shelfbot_clamp_jaw_link.step
│   ├── shelfbot_manip_proximal_link.step
│   └── shelfbot_gripper_link.step
├── stl/
│   ├── shelfbot_base_link.stl
│   ├── shelfbot_clamp_arm_link.stl
│   ├── shelfbot_clamp_jaw_link.stl
│   ├── shelfbot_manip_proximal_link.stl
│   ├── shelfbot_gripper_link_visual.stl
│   └── shelfbot_gripper_link_collision.stl
├── fcstd/
│   └── *.FCStd
└── shelfbot-3d-model-plan.md
```

---

## Dimension Reference Table

All values are estimates from the image using the ~250 mm shampoo bottle as scale. Every script uses named constants so these can be updated in one place.

| Link | Key Dimension | Estimate |
|---|---|---|
| base_link | Width × Depth × Height | 340 × 200 × 260 mm |
| base_link | Shell wall thickness | 4 mm |
| clamp_arm_link | Length | 380 mm |
| clamp_arm_link | Section W × H | 30 × 20 mm |
| clamp_arm_link | Angle from body vertical | ~45° |
| clamp_jaw_link | Housing outer diameter | 60 mm |
| clamp_jaw_link | Housing length | 40 mm |
| clamp_jaw_link | Shelf beam slot size | 40 × 40 mm |
| manip_proximal_link | Length | 160 mm |
| manip_proximal_link | Section W × H | 25 × 20 mm |
| gripper_link | Jaw inner radius | 30 mm |
| gripper_link | Jaw span (open) | 80 mm |
| gripper_link | Palm block | 60 × 40 × 30 mm |

---

## Notes and Constraints

- **All scripts run headless** via `freecadcmd scripts/link_base.py` — no GUI required.
- **No hand-guessed inertia** — FreeCAD computes mass properties from geometry + density; the script must print and save these values for later URDF `<inertial>` entries.
- **Collision geometry is separate from visual geometry** for the gripper only; for all other links, the STEP geometry IS the collision geometry (simple enough shapes).
- **Axis convention**: x forward, y left, z up throughout — match ROS 2 URDF convention from the start.
- **Units**: all FreeCAD scripts work in **millimetres** internally (FreeCAD default); STEP export preserves mm units; STL export is also mm.
