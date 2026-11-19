#!/usr/bin/env bash
# Read-only ROS 2 + Gazebo inspection; no robot goals or motion.
set -uo pipefail
source "$HOME/mrs_tray_ws/tray_env.sh"
WORLD='milestone2_multi_robot_transfer'
echo '=== ROS 2 actions (navigation, MoveIt and Lite 6) ==='
ros2 action list -t | grep -E 'navigate_to_pose|move_action|follow_joint_trajectory' || true
echo; echo '=== ROS 2 planning / Gazebo services ==='
ros2 service list | grep -E 'plan_kinematic_path|/set_pose|/get_planning_scene' || true
echo; echo '=== Gazebo models in the recorded world ==='
gz service -s "/world/$WORLD/scene/info" --reqtype gz.msgs.Empty \
  --reptype gz.msgs.Scene --timeout 8000 --req '' \
  | grep -E 'name: "(lite6|turtlebot4|pick_block|standard_dock)"' || true
echo; echo '=== Clock publisher count ==='
ros2 topic info /clock -v | grep 'Publisher count' || true
echo; echo '=== Lite 6 controllers ==='
ros2 control list_controllers -c /lite6/controller_manager || true
