#!/usr/bin/env python3
"""Spawn UFACTORY Lite 6 + Lite gripper into the ALREADY-RUNNING Milestone 2 Gazebo world.

Self-contained add-on launch file intended for mrs_tray_demo/launch/.
No edits to TurtleBot, Lite 6, Milestone 2, or overlay source packages.
Uses installed xArm xacro and controller parameter generator.
"""

from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro
from uf_ros_lib.uf_robot_utils import generate_ros2_control_params_temp_file

WORLD = 'milestone2_multi_robot_transfer'
MODEL = 'lite6'
NAMESPACE = 'lite6'
# Variant world: lower station top is z=0.40 m (center=0.20, height=0.40).
# Spawn the Lite 6 near the R1 edge of the table with 90-degree yaw.
X, Y, Z, YAW = '-0.18', '0.0', '0.40', '1.57079632679'


def launch_setup(context, *args, **kwargs):
    # Check for a running world and prevent accidental duplicate arm insertion.
    probe = subprocess.run(['gz', 'model', '--list'], text=True,
                           capture_output=True, timeout=12)
    if probe.returncode != 0 or 'Requesting state for world [' + WORLD + ']' not in probe.stdout:
        raise RuntimeError('The existing Milestone 2 Gazebo world is not running or is not the selected world. No changes made.')
    if any(line.strip() == '- ' + MODEL for line in probe.stdout.splitlines()):
        raise RuntimeError('Gazebo already contains a model named lite6. Refusing to spawn a duplicate.')

    base = Path.home() / 'mrs_tray_ws' / 'generated'
    base.mkdir(parents=True, exist_ok=True)
    control_share = Path(get_package_share_directory('xarm_controller'))
    description_share = Path(get_package_share_directory('xarm_description'))
    original_controller_yaml = control_share / 'config' / 'lite6_controllers.yaml'
    if not original_controller_yaml.is_file():
        raise RuntimeError('Installed Lite 6 control YAML missing: ' + str(original_controller_yaml))

    # UFACTORY's own config utility merges the Lite gripper controllers and
    # namespaces them separately from TurtleBot's /controller_manager.
    generated_temp = generate_ros2_control_params_temp_file(
        str(original_controller_yaml), prefix='', add_gripper=True,
        ros_namespace=NAMESPACE, update_rate=250,
        use_sim_time=True, robot_type='lite')
    controller_yaml = base / 'lite6_control_generated.yaml'
    shutil.copyfile(generated_temp, controller_yaml)

    xacro_file = description_share / 'urdf' / 'xarm_device.urdf.xacro'
    mappings = {
        'robot_type': 'lite', 'dof': '6', 'add_gripper': 'true',
        'add_realsense_d435i': 'false',
        'ros2_control_plugin': 'gz_ros2_control/GazeboSimSystem',
        'ros2_control_params': str(controller_yaml),
    }
    robot_xml = xacro.process_file(str(xacro_file), mappings=mappings).toxml()
    root = ET.fromstring(robot_xml)
    plugins = [node for node in root.findall('.//gazebo/plugin')
               if 'gz_ros2_control' in node.attrib.get('filename', '')]
    if len(plugins) != 1:
        raise RuntimeError('Expected one gz_ros2_control Gazebo plugin, found ' + str(len(plugins)))
    plugin = plugins[0]
    ros = plugin.find('ros')
    if ros is None:
        ros = ET.SubElement(plugin, 'ros')
    ET.SubElement(ros, 'namespace').text = '/' + NAMESPACE
    if plugin.find('robot_param') is None:
        ET.SubElement(plugin, 'robot_param').text = 'robot_description'
    if plugin.find('robot_param_node') is None:
        ET.SubElement(plugin, 'robot_param_node').text = 'robot_state_publisher'
    robot_xml = ET.tostring(root, encoding='unicode')

    # Experiment: register the passive gripper mimic joint
    # in the existing Gazebo gripper hardware component.
    urdf_root = ET.fromstring(robot_xml)

    gripper_components = [
        control for control in urdf_root.findall("ros2_control")
        if any(
            joint.get("name") == "finger_joint1"
            for joint in control.findall("joint")
        )
    ]

    if len(gripper_components) != 1:
        raise RuntimeError(
            "Expected exactly one gripper hardware component"
        )

    gripper_control = gripper_components[0]

    if not any(
        joint.get("name") == "finger_joint2"
        for joint in gripper_control.findall("joint")
    ):
        mimic = ET.SubElement(
            gripper_control,
            "joint",
            {"name": "finger_joint2", "mimic": "true"}
        )
        ET.SubElement(
            mimic, "state_interface", {"name": "position"}
        )
        ET.SubElement(
            mimic, "state_interface", {"name": "velocity"}
        )

    robot_xml = ET.tostring(
        urdf_root, encoding="unicode"
    )

    urdf_file = base / 'lite6_milestone2_generated.urdf'
    urdf_file.write_text(robot_xml)
    print('\n[LITE6 MILESTONE2] Generated files only in', base)
    print('[LITE6 MILESTONE2] Spawning at xyz:', X, Y, Z, 'yaw:', YAW)
    print('[LITE6 MILESTONE2] Using independent controller manager: /lite6/controller_manager\n')

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', namespace=NAMESPACE, output='screen',
        parameters=[
            {'robot_description': ParameterValue(robot_xml, value_type=str)},
            {'use_sim_time': True},
        ],
    )
    spawn = Node(
        package='ros_gz_sim', executable='create', name='spawn_lite6',
        output='screen',
        arguments=[
            '-world', WORLD, '-file', str(urdf_file), '-name', MODEL,
            '-x', X, '-y', Y, '-z', Z, '-Y', YAW,
        ],
    )
    controller_names = ['joint_state_broadcaster',
                        'lite6_traj_controller',
                        'lite_gripper_traj_controller']
    spawners = [
        Node(package='controller_manager', executable='spawner',
             name='spawn_' + name, output='screen',
             arguments=[name, '--controller-manager', '/lite6/controller_manager',
                        '--controller-manager-timeout', '90'])
        for name in controller_names
    ]
    after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=spawners)
    )
    return [robot_state_publisher, spawn, after_spawn]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
