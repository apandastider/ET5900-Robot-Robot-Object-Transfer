#!/usr/bin/env python3
"""MoveIt2 plan-only for the ALREADY RUNNING Milestone 2 Lite6 in Gazebo.

Does not launch Gazebo, ros2_control, robot_state_publisher, or a robot.
All planning is disabled for physical trajectory execution by default.
Uses the same generated URDF as ~/mrs_tray_ws launch and the installed
UFACTORY SRDF/kinematics/OMPL configuration.
"""

from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    generated = Path.home() / 'mrs_tray_ws' / 'generated'
    urdf = generated / 'lite6_milestone2_generated.urdf'
    control_yaml = generated / 'lite6_control_generated.yaml'
    if not urdf.is_file() or not control_yaml.is_file():
        raise RuntimeError('Missing generated Lite 6 URDF or controller YAML. Do not respawn. Check ~/mrs_tray_ws/generated first.')

    # The existing gz_ros2_control joint_state_broadcaster should publish here.
    # Abort if the live model is not providing its state; never start a fake robot.
    result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True, timeout=15)
    if result.returncode != 0:
        raise RuntimeError('Cannot check ROS 2 topics. Is the environment sourced?')
    topics = set(result.stdout.splitlines())
    if '/lite6/joint_states' not in topics:
        raise RuntimeError('Expected /lite6/joint_states is missing. Check ros2 topic list | grep joint_states first.')

    tf = '/lite6/tf' if '/lite6/tf' in topics else '/tf'
    tf_static = '/lite6/tf_static' if '/lite6/tf_static' in topics else '/tf_static'
    remaps = [('/joint_states', '/lite6/joint_states'),
              ('/tf', tf), ('/tf_static', tf_static)]
    print('[MRS MoveIt] Using joint states /lite6/joint_states; TF:', tf, tf_static)
    print('[MRS MoveIt] PLAN ONLY: no controller commands / Gazebo launches')

    # Loads installed Lite 6 planning model, SRDF, joint limits, IK and OMPL.
    builder = MoveItConfigsBuilder(
        context=context,
        robot_type='lite', dof='6', add_gripper=True,
        limited=True, hw_ns='lite6',
        controllers_name='fake_controllers',
        ros2_control_plugin='gz_ros2_control/GazeboSimSystem',
        ros2_control_params=str(control_yaml),
        add_realsense_d435i=False,
    )
    config = builder.to_moveit_configs().to_dict()
    config['robot_description'] = urdf.read_text()  # Match Gazebo exactly.
    config['use_sim_time'] = True
    config['allow_trajectory_execution'] = False
    config['moveit_manage_controllers'] = False
    config['publish_robot_description_semantic'] = True
    config['publish_robot_description'] = True

    move_group = Node(
        package='moveit_ros_move_group', executable='move_group',
        name='move_group', output='screen',
        parameters=[config], remappings=remaps,
    )

    moveit_share = Path(get_package_share_directory('xarm_moveit_config'))
    rviz_file = moveit_share / 'rviz' / 'moveit.rviz'
    if not rviz_file.is_file():
        raise RuntimeError(f'Installed MoveIt RViz config missing: {rviz_file}')
    rviz_params = {k: v for k, v in config.items() if k in (
        'robot_description', 'robot_description_semantic',
        'robot_description_kinematics', 'robot_description_planning',
        'planning_pipelines', 'default_planning_pipeline', 'ompl')}
    rviz_params['use_sim_time'] = True
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', str(rviz_file)], parameters=[rviz_params],
        remappings=remaps,
    )
    return [move_group, rviz]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
