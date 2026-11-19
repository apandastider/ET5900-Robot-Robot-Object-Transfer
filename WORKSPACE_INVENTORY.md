# Collected original workspace files

Collected: 2026-11-19

Files copied: 37; skipped: 2.
The collector did not modify any original ROS workspace files.

## Map copy

Copied milestone2_map.yaml plus referenced image milestone2_map.pgm

## Copied

- `src/mrs_tray_demo/CMakeLists.txt`
- `src/mrs_tray_demo/launch/lite6_low_station_fixed_grasp.launch.py`
- `src/mrs_tray_demo/launch/lite6_low_station_harmonic.launch.py`
- `src/mrs_tray_demo/launch/lite6_low_station_mimic_flag_test.launch.py`
- `src/mrs_tray_demo/launch/lite6_low_station_mimic_test.launch.py`
- `src/mrs_tray_demo/launch/lite6_low_station_spawn.launch.py`
- `src/mrs_tray_demo/launch/lite6_milestone2_spawn.launch.launch.py.bak`
- `src/mrs_tray_demo/launch/lite6_milestone2_spawn.launch.py`
- `src/mrs_tray_demo/launch/lite6_moveit_tray_only.launch.py`
- `src/mrs_tray_demo/launch/tray_robot_description.launch.py`
- `src/mrs_tray_demo/launch/tray_tb4_spawn.launch.py`
- `src/mrs_tray_demo/models/pick_block.sdf`
- `src/mrs_tray_demo/package.xml`
- `src/mrs_tray_demo/urdf/tb4_lite_tray.urdf.xacro`
- `src/mrs_tray_demo/worlds/milestone2_tray_low_station.sdf`
- `scripts/add_orange_block_to_moveit.py`
- `scripts/add_orange_block_to_moveit_backup.py`
- `scripts/load_lite6_virtual_grasp.py`
- `scripts/print_lite6_planned_joints.py`
- `scripts/tray_pick_place_step.py`
- `scripts/tray_transfer_fsm.py`
- `scripts/tray_transfer_fsm_backup.py`
- `scripts/virtual_grasp_hold.py`
- `scripts/virtual_grasp_hold_backup.py`
- `scripts/virtual_grasp_hold_old.py`
- `config/finger_velocity_probe.yaml`
- `config/lite6_gripper_official_test.yaml`
- `config/lite6_gripper_tray.yaml`
- `config/lite6_gripper_tray.yaml.bak`
- `config/lite6_gripper_tray.yaml.before_fix`
- `config/tray_localization.yaml`
- `config/tray_nav2.yaml`
- `generated/lite6_control_generated.yaml`
- `generated/lite6_milestone2_generated.urdf`
- `tray_env.sh`
- `maps/milestone2_map.pgm (referenced by saved map)`
- `maps/milestone2_map.yaml`

## Skipped

- `src/mrs_tray_demo/launch/__pycache__` — build/cache/private directory
- `scripts/__pycache__` — build/cache/private directory

## Before publishing

- Check that the world, model SDF, launch files, local environment, latest FSM and grasp helper are included.
- Inspect generated URDF/controller configuration and launch files for machine-specific paths.
- Verify map file/image and license of third-party assets.
- Document external xarm/TurtleBot workspace commits and any local patches.
- Inspect the git diff and audit output before pushing.
