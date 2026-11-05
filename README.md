# ShelfBot — Autonomous Shelf-Restocking Robot

![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-blue.svg)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange.svg)
![License](https://img.shields.io/badge/License-Apache%202.0-green.svg)
![Python](https://img.shields.io/badge/Python-3.10%2B-blue.svg)
![FreeCAD](https://img.shields.io/badge/FreeCAD-1.x-yellow.svg)

> **ShelfBot** is an autonomous shelf-restocking robot startup and PhD research project (founder: **Rohini Narayan Koli**, Berlin). This repository contains the complete ROS 2 simulation workspace, FreeCAD-based 3D mesh generation scripts, and launch configurations for Gazebo Harmonic simulation.

---

## 🎯 Project Overview

ShelfBot is a wheeled mobile robot with a telescoping mast and clamp arm designed to navigate retail store aisles, climb shelf uprights, and restock products. The first-generation hardware focuses on a wheeled base + mast/lift; rack-climbing is experimental (simulation phase).

### Key Capabilities (Simulation)
- **Gazebo Harmonic (gz-sim8)** simulation with realistic shelf world
- **ROS 2 Jazzy** workspace with two packages: `shelfbot_description` + `shelfbot_sim`
- **FreeCAD Python scripting** for reproducible, version-controlled 3D mesh generation
- **5-link kinematic chain** with accurate mass properties from CAD
- **Climb FSM controller** for shelf-level transitions
- **Shelf gap detection** (separate launch to avoid bridge conflicts)

---

## 📁 Repository Structure

```
shelfbot_simulation_research/
├── ros2_ws/                          # ROS 2 Jazzy workspace
│   └── src/
│       ├── shelfbot_description/     # URDF/xacro + meshes (visual/collision)
│       │   ├── urdf/shelfbot.urdf.xacro
│       │   ├── meshes/visual|collision/*.stl
│       │   ├── launch/display.launch.py
│       │   └── package.xml
│       └── shelfbot_sim/             # Gazebo worlds + launch + controllers
│           ├── launch/
│           │   ├── gazebo.launch.py      # Main simulation launch
│           │   └── shelf_detector.launch.py
│           ├── worlds/shelfbot_store_aisle.sdf
│           ├── config/gz_bridge.yaml
│           ├── shelfbot_sim/
│           │   ├── climb_controller.py   # FSM-based shelf climb
│           │   └── shelf_detector.py     # Gap detection
│           └── package.xml
│
├── shelfbot-3d/                      # FreeCAD 3D modeling (source of truth)
│   ├── scripts/
│   │   ├── link_base.py
│   │   ├── link_clamp_arm.py
│   │   ├── link_clamp_jaw.py
│   │   ├── link_manip_proximal.py
│   │   ├── link_gripper.py
│   │   └── assemble_check.py
│   ├── step/                         # STEP AP214 exports
│   ├── stl/                          # STL exports (visual + collision)
│   ├── fcstd/                        # Native FreeCAD files
│   ├── mass_props/                   # Mass property JSON outputs
│   └── shelfbot-3d-model-plan.md     # Detailed modeling plan
│
├── .env                              # Gitignored - API keys (create locally)
├── .gitignore
└── README.md                         # This file
```

---

## 🤖 Robot Kinematics

```
base_link (central body, ~2.1 kg)
├── clamp_arm_link (diagonal arm, ~0.7 kg)  ──► clamp_jaw_link (rail gripper, ~0.15 kg)
└── manip_proximal_link (front arm, ~0.3 kg) ──► gripper_link (product gripper, ~0.5 kg)
```

**Total mass**: ~3.2 kg  
**Envelope (assembled)**: 370 × 260 × 599 mm

All inertia values come from FreeCAD mass-property computation (ABS plastic, 1050 kg/m³, 4 mm shell), not hand-guessed.

---

## 🚀 Quick Start

### Prerequisites
- **Ubuntu 24.04** (Noble) with **ROS 2 Jazzy**
- **Gazebo Harmonic** (`gz-sim8`)
- **FreeCAD 1.x** (Flatpak: `org.freecad.FreeCAD`)
- Python 3.10+ with `python-dotenv`, `trimesh`

### 1. Clone & Setup
```bash
git clone https://github.com/Rohini341999/shelfbot_simulation_research.git
cd shelfbot_simulation_research
```

### 2. Build ROS 2 Workspace
```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 3. Generate 3D Meshes (FreeCAD)
```bash
# From repo root (not ros2_ws)
cd ../shelfbot-3d/scripts

# Generate each link (run individually)
flatpak run org.freecad.FreeCAD --console link_base.py
flatpak run org.freecad.FreeCAD --console link_clamp_arm.py
flatpak run org.freecad.FreeCAD --console link_clamp_jaw.py
flatpak run org.freecad.FreeCAD --console link_manip_proximal.py
flatpak run org.freecad.FreeCAD --console link_gripper.py

# Assembly validation
flatpak run org.freecad.FreeCAD --console assemble_check.py

# Copy STLs to ROS package
cp ../stl/*.stl ../../ros2_ws/src/shelfbot_description/meshes/visual/
cp ../stl/*.stl ../../ros2_ws/src/shelfbot_description/meshes/collision/
```

### 4. Run Simulation

**Terminal 1 — Gazebo + Robot + Climb Controller:**
```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch shelfbot_sim gazebo.launch.py
```

**Terminal 2 — Shelf Detector (after ~10 s):**
```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch shelfbot_sim shelf_detector.launch.py
```

### 5. Drive & Test
```bash
# Drive forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3}}' --once

# Trigger climb to shelf level (height in metres)
ros2 topic pub /climb_controller/start std_msgs/msg/Float32 '{data: 0.55}' --once

# Watch climb state
ros2 topic echo /climb_controller/state
```

---

## 📐 3D Modeling (FreeCAD)

All links are generated via **headless FreeCAD Python scripts** — no manual GUI work. Each script:

1. Defines named constants at top for easy re-scaling
2. Builds geometry using `Part` workbench
3. Computes mass properties (mass, CoM, inertia tensor) at given density
4. Exports `.step` (AP214), `.stl`, and prints properties to stdout
5. Runs via `flatpak run org.freecad.FreeCAD --console script.py`

### Regenerating Meshes
After modifying any `link_*.py`:
```bash
flatpak run org.freecad.FreeCAD --console link_<name>.py
cp ../stl/shelfbot_<name>_link.stl ../../ros2_ws/src/shelfbot_description/meshes/visual/
cp ../stl/shelfbot_<name>_link.stl ../../ros2_ws/src/shelfbot_description/meshes/collision/
cd ../../ros2_ws && colcon build --symlink-install --packages-select shelfbot_description
```

### Assembly Check
```bash
flatpak run org.freecad.FreeCAD --console assemble_check.py
# Outputs: shelfbot_assembly_check.step, shelfbot_assembly_summary.json
```

---

## ⚙️ Simulation Details

### Launch Files
| Launch File | Description |
|-------------|-------------|
| `gazebo.launch.py` | Full simulation: Gazebo world, RSP, spawn, ros_gz_bridge, climb_controller |
| `shelf_detector.launch.py` | Gap detection node (launch separately after Gazebo is up) |
| `display.launch.py` | RViz2 only (no Gazebo) — `ros2 launch shelfbot_description display.launch.py` |

### Fidelity Levels (Launch Parameter)
Controlled via launch arguments (not separate branches):
- **L0**: Differential drive only, no climb
- **L1**: Climb via attach/detach joint plugin (stable fallback)
- **L2**: Full contact gripping (experimental, may be cut)

### Critical URDF/Gazebo Fixes (Already Applied)
- `GZ_SIM_RESOURCE_PATH` set to parent of package share dir (first action in LaunchDescription)
- `robot_description` wrapped in `ParameterValue(..., value_type=str)` for Jazzy
- All STL meshes use `scale="0.001 0.001 0.001"` (STLs are in mm)
- `base_link` visual origin at body geometric centre (`xyz="0 0 ${body_h/2}"`)
- `camera_optical_link` has minimal `<inertial>` (mass=0.001) for DART entity map
- Shelf SDF is a **single link** (avoids Ogre2/DART crashes)
- Spawn Z = wheel_radius (0.076 m)
- TimerAction delays: Gazebo 5s → Bridge 6s → Nodes 7s

---

## 🔐 Security

- **Pre-commit & pre-push hooks** scan for secrets (API keys, tokens, passwords)
- `.env` file is gitignored — store NVIDIA/OpenAI API keys there
- Never commit real credentials

```bash
# Bypass hooks (false positives only!)
git commit --no-verify
git push --no-verify
```

---

## 📚 Documentation

| File | Description |
|------|-------------|
| `shelfbot-3d/shelfbot-3d-model-plan.md` | Complete 3D modeling plan with all 5 links |
| `ros2_ws/src/shelfbot_sim/launch/gazebo.launch.py` | Heavily commented launch file with gotchas |
| `AGENTS.md` (repo root) | Agent instructions for this codebase |
| `schedule/` | 14-week C++ ROS 2 simulation plan (70 sessions) |

---

## 📄 License

Apache License 2.0 — see `LICENSE` (to be added) or individual `package.xml` files.

---

## 🙏 Acknowledgments

- ROS 2 Jazzy & Gazebo Harmonic teams
- FreeCAD community for scriptable CAD
- Open Robotics for `ros_gz` bridge
- trimesh for mesh processing utilities