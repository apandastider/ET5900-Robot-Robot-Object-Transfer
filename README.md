# ET5900 — Robot–Robot Object Transfer

**Simulation demonstration:** TurtleBot 4 Lite → UFACTORY Lite 6 → receiving area, coordinated with a simple ROS 2 finite-state machine (FSM).

[Project repository](https://github.com/apandastider/ET5900-Robot-Robot-Object-Transfer) · ROS 2 **Jazzy** · Gazebo Sim **8** · MoveIt 2 · Nav2

> This repository documents a working course demonstration / engineering experiment, **not a student assignment**. It contains no assignment prompts, grading rubric, or student deliverables.

## Overview

A TurtleBot 4 Lite begins at its map-frame home pose with an orange block (`pick_block`) on its tray. Nav2 brings the TurtleBot to a transfer station, where a fixed UFACTORY Lite 6 performs collision-aware planned movements. A demonstration-only virtual grasp makes the block follow the arm; the arm lifts and places the block in the receiving area. The TurtleBot then navigates to a final location.

```text
TurtleBot home (-3.00, -1.60)
           │ orange block on mobile tray
           ▼
Nav2 transfer station (-0.40, +0.15)
           │
           ▼
Lite 6 PREGRASP → virtual ATTACH → LIFT → PLACE → RELEASE → RETRACT
           │
           ▼
TurtleBot final Nav2 goal (-2.30, +1.45) → DONE
```

This is **a mobile robot–fixed manipulator transfer**. The orange block is released at the receiving area **before** TurtleBot navigates to the final location. That final TurtleBot goal is not the block's placement position.

## Recorded motion targets

| Platform / state | Frame | Target |
|---|---|---|
| TurtleBot initial pose | `map` | `x=-3.00, y=-1.60, yaw=0` |
| TurtleBot transfer goal | `map` | `x=-0.40, y=+0.15, yaw=0` |
| TurtleBot final goal | `map` | `x=-2.30, y=+1.45, yaw=0` |
| Lite 6 pregrasp | joints 1–6, rad | `[1.2829, 1.4382, 2.1537, 0.0000, 0.7154, 1.2828]` |
| Lite 6 lift | joints 1–6, rad | `[0.9561, 0.4754, 1.5481, 0.0000, 1.0727, 0.9560]` |
| Lite 6 placement | joints 1–6, rad | `[-0.4864, 0.5401, 0.6637, -0.0001, 0.1236, -0.4862]` |

**Important:** These coordinates and joint angles apply to the recorded Gazebo world, map alignment, robot model, tray geometry, and collision-scene configuration. They are not portable calibration values.

## Installation and Setup

### 1. System Requirements

- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- Gazebo Sim 8
- MoveIt 2
- Navigation2 (Nav2)
- TurtleBot 4 ROS 2 packages
- UFACTORY Lite 6 / xarm ROS 2 packages

See [Dependencies](docs/DEPENDENCIES.md) for additional setup requirements.

### 2. Clone the Repository

Open an Ubuntu terminal:

```bash
cd ~

git clone https://github.com/apandastider/ET5900-Robot-Robot-Object-Transfer.git ~/mrs_tray_ws

cd ~/mrs_tray_ws
```

**Note:** These commands are for a fresh installation. Do not clone over an existing `~/mrs_tray_ws` directory.

### 3. Build the ROS 2 Workspace

First install and build the upstream TurtleBot 4 and UFACTORY Lite 6 dependencies described in `docs/DEPENDENCIES.md`.

Then build this project:

```bash
source /opt/ros/jazzy/setup.bash

cd ~/mrs_tray_ws

colcon build --symlink-install
```

### 4. Source the Workspace

```bash
source ~/mrs_tray_ws/tray_env.sh
```

The environment script is configured for the original project workspace layout and required ROS 2 overlays. Review its paths when installing on a different machine.

### 5. Run the Simulation

The complete step-by-step launch instructions, including Gazebo, TurtleBot 4, Lite 6, MoveIt 2, Nav2, the orange block, and the virtual grasp bridge, are available in:

[Full Simulation Guide](docs/RUN_DEMO.md)

### 6. Run the FSM

After launching the simulation components, verify readiness:

```bash
source ~/mrs_tray_ws/tray_env.sh

python3 ~/mrs_tray_ws/scripts/tray_transfer_fsm.py --check-only
```

Execute the complete robot-to-robot transfer demonstration:

```bash
python3 -u ~/mrs_tray_ws/scripts/tray_transfer_fsm.py \
  2>&1 | tee ~/mrs_tray_ws/tray_transfer_demo.log
```

The FSM coordinates TurtleBot navigation, Lite 6 pregrasp, virtual attachment, lifting, placement, release, retraction, and final navigation.

**Note:** The current grasp uses simulated pose-following rather than a physical fixed joint.

## Repository layout

```text
ET5900-Robot-Robot-Object-Transfer/
├── README.md
├── src/                        ROS 2 packages, launch files, SDF worlds and models
│   └── mrs_tray_demo/           populated from the working Ubuntu workspace
├── scripts/                    working FSM and grasp/support Python scripts
├── config/                     recorded poses, AMCL/Nav2 and robot settings
├── maps/                       saved map YAML and image, when collected
├── generated/                  Gazebo-matching URDF/controller config (review first)
├── tools/                      scene setup, environment check and collection tools
└── docs/
    ├── RUN_DEMO.md             full terminal-by-terminal restart instructions
    ├── DEPENDENCIES.md         software and upstream workspaces
    ├── PUBLISH_TO_GITHUB.md    safe upload instructions for this exact GitHub URL
    └── TROUBLESHOOTING.md      common issues and limitations
```

### Repository source completeness

The **working Gazebo world, custom ROS 2 package, and some runtime scripts exist on the Ubuntu workstation** in `~/mrs_tray_ws`. This downloadable repository package includes the latest available FSM and documentation; it **must be completed from the workstation** before being described as a full reproducible source repository. The collector copies the *actual working files* rather than inventing missing launch files or robot descriptions.

On the Ubuntu workstation:

```bash
# Unzip this repository folder OUTSIDE ~/mrs_tray_ws, then run from its root:
bash tools/collect_from_workstation.sh
bash tools/audit_repo.sh
```

Do not publish until the audit reports all required source/config files present, and you have inspected generated files for absolute paths, credentials and licensing.

## Run the simulation

Follow **[docs/RUN_DEMO.md](docs/RUN_DEMO.md)** to start Gazebo, Lite 6, TurtleBot, the `/clock` and `SetEntityPose` bridges, MoveIt, AMCL and Nav2, then:

```bash
source ~/mrs_tray_ws/tray_env.sh
python3 ~/mrs_tray_ws/scripts/tray_transfer_fsm.py --check-only
python3 -u ~/mrs_tray_ws/scripts/tray_transfer_fsm.py \
  2>&1 | tee ~/mrs_tray_ws/tray_transfer_demo.log
```

FSM states are also published on `/tray_transfer_fsm/state` for recording overlays.

### Actual grasp behavior and its limitation

The current `virtual_grasp_hold.py` repeatedly calls Gazebo `/world/milestone2_multi_robot_transfer/set_pose` to move `pick_block` as the Lite 6 tool moves (the latest requested rate is **15 Hz**). This is **not a rigid or force-controlled grasp**. Gravity and collision physics between updates can make the orange block jump or jitter. Increasing update frequency may help visually, but a Gazebo `DetachableJoint` or another verified physical attachment is the appropriate future improvement.

## Reproducibility and verification

The FSM's `--check-only` has passed in the original Jazzy/Gazebo setup: models, `/navigate_to_pose`, MoveIt `/plan_kinematic_path`, and `/lite6/lite6_traj_controller/follow_joint_trajectory` were detected. A later run reported successful navigation, a 71-point pregrasp trajectory, and virtual grasp activation; additional interaction fixes were applied. The instructor has reported overall execution working with visible block jitter. This packaged snapshot has **not** been re-tested on another computer.

No physical robot should be controlled using these scripts without separate safety integration and validation.

## Third-party software and licensing

TurtleBot 4, Nav2, MoveIt 2, Gazebo and UFACTORY xarm ROS 2 components are third-party projects. Their install/build outputs are excluded. Check applicable upstream licenses for any copied source or assets; no new license has been applied to this repository by default.
