# Run the automatic tray-transfer demo (ROS 2 Jazzy)

**Purpose:** Reproduce the Gazebo **simulation-only** used for the Lite 6 + TurtleBot 4 Lite tray transfer. Each running launch below stays in its own terminal. Do not run a second world/robot into an already-running simulation. These commands are based on the previously-used experiment paths; the launch files and YAML assets must exist on the original workstation.

## Recorded poses

- TurtleBot start/home `(x=-3.00, y=-1.60, yaw=0)` in map frame.
- TurtleBot transfer station `(x=-0.40, y=0.15, yaw=0)` in map frame.
- TurtleBot delivery station `(x=-2.30, y=1.45, yaw=0)` in map frame.
- Lite 6 pregrasp `[1.2829, 1.4382, 2.1537, 0.0, 0.7154, 1.2828]` rad.
- Lite 6 lift `[0.9561, 0.4754, 1.5481, 0.0, 1.0727, 0.9560]` rad.
- Lite 6 placement `[-0.4864, 0.5401, 0.6637, -0.0001, 0.1236, -0.4862]` rad.

These poses are valid only for the same robot model, saved map/frame alignment, world objects, and tray geometry. They are not universal hardware coordinates.

## Before you start

Close all previous launch terminals gracefully. Do not `pkill` ROS/Gazebo processes indiscriminately; they may belong to other workspaces. In every fresh terminal use:

```bash
source ~/mrs_tray_ws/tray_env.sh
```

Ensure `~/mrs_tray_ws/scripts/virtual_grasp_hold.py` is still the tested version and the real source workspace has been built and installed previously. The FSM is a new coordinator only; it does not install or overwrite robot packages.

## Terminal 1 — Gazebo world

```bash
source ~/mrs_tray_ws/tray_env.sh
gz sim -r -v 4 \
  ~/mrs_tray_ws/src/mrs_tray_demo/worlds/milestone2_tray_low_station.sdf
```

Start only one world with the name `milestone2_multi_robot_transfer`.

## Terminal 2 — Clock bridge (unless /clock already exists)

```bash
source ~/mrs_tray_ws/tray_env.sh
ros2 run ros_gz_bridge parameter_bridge \
  '/world/milestone2_multi_robot_transfer/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' \
  --ros-args \
  -r /world/milestone2_multi_robot_transfer/clock:=/clock \
  -r __node:=milestone2_clock_bridge
```

Verify in another sourced terminal: `ros2 topic hz /clock`. Keep **one** publisher for `/clock` (`ros2 topic info /clock -v`). If a launch already provides it, skip this separate terminal.

## Terminal 3 — Lite 6

```bash
source ~/mrs_tray_ws/tray_env.sh
ros2 launch mrs_tray_demo lite6_low_station_spawn.launch.py
```

This is the previously used low-station Lite 6 spawn. Do not also run `lite6_low_station_fixed_grasp.launch.py` unless you deliberately replace this launch with that version; two spawns would duplicate the arm. The FSM uses your working `virtual_grasp_hold.py` **ROS service bridge**, not the detachable-joint topic plugin.

## Terminal 4 — TurtleBot 4 Lite

```bash
source ~/mrs_tray_ws/tray_env.sh
ros2 launch mrs_tray_demo tray_tb4_spawn.launch.py \
  model:=lite \
  world:=milestone2_multi_robot_transfer \
  x:=-3.0 y:=-1.6 yaw:=0.0 \
  nav2:=false localization:=false
```

## Scene preparation — after spawning the TurtleBot (BEFORE the FSM)

The orange block **starts on the TurtleBot tray at home** `(x=-3.00, y=-1.60)`. It is picked up by the Lite 6 after the TurtleBot drives to the transfer pose. The Lite 6 deposits it at the receiving area; the TurtleBot then drives to the final pose `(x=-2.30, y=1.45)`.

Run once in a separate sourced terminal:

```bash
bash tools/prepare_demo_scene.sh
```

This checks for `pick_block`, spawns it **only if absent** using `mrs_tray_demo/models/pick_block.sdf` at `(x=-2.97, y=-1.60, z=0.40)`, and removes the `standard_dock` Gazebo model if it exists. The block should rest on the TurtleBot tray; inspect visually and confirm `gz model -m pick_block -p`. Do not recreate a duplicate or reset a block after the robot has already moved.

Equivalent individual commands are:

```bash
gz service -s /world/milestone2_multi_robot_transfer/remove \
  --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean \
  --timeout 3000 --req 'name: "standard_dock", type: MODEL'

ros2 run ros_gz_sim create -world milestone2_multi_robot_transfer \
  -file "$(ros2 pkg prefix --share mrs_tray_demo)/models/pick_block.sdf" \
  -name pick_block -x -2.97 -y -1.60 -z 0.40
```

Only use the individual block spawn command if `pick_block` is absent.

## Terminal 5 — Gazebo pose service bridge (virtual grasp requirement)

```bash
source ~/mrs_tray_ws/tray_env.sh
ros2 run ros_gz_bridge parameter_bridge \
  /world/milestone2_multi_robot_transfer/set_pose@ros_gz_interfaces/srv/SetEntityPose
```

Check `ros2 service list | grep /world/milestone2_multi_robot_transfer/set_pose`.

## Terminal 6 — MoveIt 2 planner for already-running Lite 6

```bash
source ~/mrs_tray_ws/tray_env.sh
ros2 launch mrs_tray_demo lite6_moveit_tray_only.launch.py
```

This plan-only MoveIt launch does **not** drive Gazebo itself. The FSM asks MoveIt to plan and then sends its planned joint trajectory to the ROS 2 controller. Keep the planning scene collision objects (table, tray, etc.) correctly configured. The FSM removes the stationary `orange_pick_block` representation in MoveIt after the virtual pickup, if the planning-scene service exists. This does not remove the Gazebo `pick_block` model.

## Terminal 7 — Map + AMCL localization

```bash
source ~/mrs_tray_ws/tray_env.sh
ros2 launch turtlebot4_navigation localization.launch.py \
  map:=$HOME/mrs_milestone2_ws/src/mrs_milestone2/maps/milestone2_map.yaml \
  params:=$HOME/mrs_tray_ws/config/tray_localization.yaml \
  use_sim_time:=true
```

After AMCL and the map server are active, initialize at the **actual spawned home** location (run in any separate sourced terminal). If you moved the TurtleBot, change the initial pose to its true pose rather than blindly resetting it:

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  '{header: {frame_id: map}, pose: {pose: {position: {x: -3.0, y: -1.6, z: 0.0}, orientation: {w: 1.0}}, covariance: [0.25, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0, 0, 0.01, 0, 0, 0, 0, 0, 0, 0.01, 0, 0, 0, 0, 0, 0, 0.01, 0, 0, 0, 0, 0, 0, 0.0685]}}'
```

In RViz, verify the TurtleBot's initial pose is correctly aligned with the saved map. The FSM will not guess AMCL's origin.

## Terminal 8 — Nav2

```bash
source ~/mrs_tray_ws/tray_env.sh
ros2 launch turtlebot4_navigation nav2.launch.py \
  params_file:=$HOME/mrs_tray_ws/config/tray_nav2.yaml \
  use_sim_time:=true
```

Do not launch Nav2 a second time if it was already included by another launch.

## Terminal 9 — Readiness check, then run

Copy script from the ZIP onto your workstation (or run it directly from the extracted directory):

```bash
mkdir -p ~/mrs_tray_ws/scripts
source ~/mrs_tray_ws/tray_env.sh
```

Verify interfaces:

```bash
ros2 action list -t | grep -E 'navigate_to_pose|lite6_traj_controller'
ros2 service list -t | grep -E 'plan_kinematic_path|set_pose'
ros2 topic hz /clock
ros2 topic echo /lite6/joint_states --once
```

Read-only FSM check:

```bash
python3 ~/mrs_tray_ws/scripts/tray_transfer_fsm.py --check-only
```

**Start your screen/video recording**, then run:

```bash
python3 -u ~/mrs_tray_ws/scripts/tray_transfer_fsm.py \
  | tee ~/mrs_tray_ws/tray_transfer_demo.log
```

The FSM automatically runs:

`CHECK -> NAVIGATE_TO_TRANSFER -> PREGRASP -> ATTACH -> LIFT -> PLACE -> RELEASE -> RETRACT -> NAVIGATE_TO_DELIVERY -> DONE`.

It aborts on failed Nav2, MoveIt, Gazebo helper, controller, or arm joint verification. The successful final Nav2 status is **not** proof the block was picked from the moving TurtleBot tray and deposited correctly at the receiving area; inspect the Gazebo scene and recording. The orange block is moved by your **demonstration-only** virtual grasp, not by actual finger/force physics.

For recording, use the Gazebo GUI full-screen with OBS Studio or your screen recorder. You can monitor the FSM message topic using another terminal:

```bash
ros2 topic echo /tray_transfer_fsm/state
```

## Troubleshooting

- **Gazebo model not found**: wrong world, Gazebo not fully started, or model names differ. Check `gz service -l`.
- **Nav2 action missing**: check `ros2 action list` and AMCL/map readiness.
- **No MoveIt service**: confirm `lite6_moveit_tray_only.launch.py` is actually running with the generated URDF.
- **Trajectory controller not found**: inspect `ros2 action list -t`. If the controller is named differently but is the **same simulator arm**, pass `--controller-action /exact/controller/follow_joint_trajectory`.
- **Virtual grasp gap > 0.16 m**: the supplied pregrasp may not be a true grasp pose. The helper intentionally refuses to attach if link6 is too far from `pick_block`; correct the pose/geometry, rather than arbitrarily increasing the gap.
- **MoveIt cannot plan**: check the static collision scene, especially the table, orange block, and moving TurtleBot tray. No automatic straight-line fallback is used.
- **Block slips off the mobile tray or falls after release**: verify that both trays/receiving surfaces have collision geometry and the recorded pickup and placement poses align with the block. The existing pose-following helper is not a detachable physical joint.
- **Arm retract path unsafe**: the script plans the retract from placement back to the recorded lift joint pose. Check the MoveIt and Gazebo trajectories. For isolated debugging you can pass `--skip-retract`, but then do not allow TurtleBot departure if the arm obstructs its path.

**Simulation only.** Do not use the script against a physical Lite 6 or TurtleBot without a separately validated safety/control design.
