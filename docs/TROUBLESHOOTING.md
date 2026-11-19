# Troubleshooting and known limitations

## FSM reports `ParameterAlreadyDeclaredException: use_sim_time`

Use the updated `scripts/tray_transfer_fsm.py`. ROS 2 Jazzy can declare `use_sim_time` automatically; the FSM sets the existing parameter rather than redeclaring it.

## FSM cannot discover `get_action_names_and_types`

The Jazzy-compatible version uses `from rclpy.action import get_action_names_and_types` and calls `get_action_names_and_types(self)`.

## Lite 6 action reported missing despite visible controller

Use `ros2 action list -t` and `ros2 control list_controllers -c /lite6/controller_manager`. The recorded Gazebo action was:

```text
/lite6/lite6_traj_controller/follow_joint_trajectory
```

An additional un-namespaced action can be present. The FSM defaults to the verified namespaced action, and has `--controller-action` for overrides. Do not start a second controller just to make the check pass.

## `gz model` cannot find the visible TurtleBot / `/gazebo/worlds` times out

Do not interpret a discovery timeout as a missing robot. Query the existing world directly:

```bash
gz service -s /world/milestone2_multi_robot_transfer/scene/info \
  --reqtype gz.msgs.Empty --reptype gz.msgs.Scene \
  --timeout 8000 --req '' \
  | grep -E 'name: "(lite6|turtlebot4|pick_block)"'
```

The latest FSM uses this `scene/info` preflight, avoiding the repeated `/gazebo/worlds` lookup. The grasp helper still queries `gz model` to get its initial pose and can need separate debugging if that service times out.

## Failure removing `orange_pick_block` from MoveIt

Gazebo `pick_block` and MoveIt `orange_pick_block` are different representations. The latest FSM queries planning-scene object IDs first; it only removes existing matching static collision IDs, and confirms they are absent afterward. It does not delete the Gazebo block.

## `rcl_shutdown already called` in grasp helper

Use `python3 tools/fix_virtual_grasp_shutdown.py` from the repository root. It modifies the *local* script with a backup, replacing an unconditional shutdown with a guarded one. This prevents the cleanup traceback after SIGINT.

## Orange block jumps during lift/placement

`virtual_grasp_hold.py` calls Gazebo `SetEntityPose` at the configured frequency while gravity and collision physics remain active. The latest FSM snapshot uses 15 Hz instead of 8 Hz, which may reduce jitter but is not a physical attachment. A Gazebo `DetachableJoint` is a future alternative; this repo does not claim to use it.

## More than one `/clock` publisher

Check `ros2 topic info /clock -v`. Keep exactly one ROS `/clock` publisher for this world. Do not start an extra bridge if an existing launch already provides it.

## Re-running after a failed trial

The TurtleBot may already be at the transfer station; the block may have fallen or been carried away. Inspect the world and return the objects to the documented start state before recording another run. Do not reset AMCL to the home pose if the actual robot has moved.

## Simulation-only caution

`--check-only` tests interface discovery, not path safety, grasp physics, final placement, or final robot localization accuracy. Confirm tray collisions, receiving surface, clearances and velocities before unattended execution. Do not direct this demo at a physical arm.
