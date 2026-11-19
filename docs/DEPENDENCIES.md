# Dependencies and environment

The experiment was developed with **Ubuntu 24.04**, **ROS 2 Jazzy**, **Gazebo Sim 8**, **TurtleBot 4 Lite**, **UFACTORY Lite 6**, **MoveIt 2**, **Navigation2**, **AMCL** and **ros_gz_bridge**.

The original system used the following local workspaces:

- `~/mrs_tray_ws` — experiment-specific ROS 2 package, world, models, scripts, configs, generated URDF/controller settings.
- `~/lite6_jazzy_ws` — UFACTORY/xarm source and build overlays required by the custom Lite 6 spawn and MoveIt model.
- `~/turtlebot4_ws` — TurtleBot 4 packages and overlays, when applicable.
- `~/mrs_milestone2_ws` — location of the previously-used saved `milestone2_map.yaml`; the repository collector copies the map and its image to `maps/` if found.

Software packages expected at runtime include `rclpy`, `nav2_msgs`, `moveit_msgs`, `control_msgs`, `geometry_msgs`, `sensor_msgs`, `std_msgs`, `action_msgs`, `ros_gz_interfaces`, `ros_gz_bridge`, `ros_gz_sim`, `tf2_ros`, `controller_manager`, and the project-specific ROS packages. These are ROS/apt/colcon dependencies, **not ordinary pip dependencies**.

The exact upstream xarm/TurtleBot commit revisions and any modifications made inside those upstream workspaces are **not available in this archive**. Record the corresponding repository URLs and commit SHAs when collecting the experiment. If your launch depends on locally modified upstream files, include your own patches with correct attribution, rather than blindly copying upstream build/install directories.

## Build and environment

Run the collector on the *original Ubuntu workstation* first, then inspect `tray_env.sh` (especially `source` order). A typical ROS 2 source workspace is built using:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/mrs_tray_ws
colcon build --symlink-install
source ~/mrs_tray_ws/tray_env.sh
```

**Do not rebuild a functioning runtime simply to upload to GitHub.** The commands above are for reproduction, after installing upstream dependencies and checking package build instructions. The original generated Lite 6 URDF and controller YAML need to match the actual spawned Gazebo robot before planning.

The custom scene uses `milestone2_multi_robot_transfer`, `lite6`, `turtlebot4`, `pick_block`, `standard_dock`, and a MoveIt planning-scene orange block potentially named `orange_pick_block`. ROS/Gazebo model names and MoveIt object IDs are separate.
