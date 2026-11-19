#!/usr/bin/env python3
import math
import re
import subprocess
from pathlib import Path
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, ObjectColor, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene

def gz_pose(model):
    p = subprocess.run(
        ['gz', 'model', '-m', model, '-p'],
        capture_output=True, text=True, timeout=20
    )
    s = p.stdout + p.stderr
    m = re.search(
        r'Pose \[ XYZ \(m\) \] \[ RPY \(rad\) \]:\s*'
        r'\[([^]]+)\]\s*\[([^]]+)\]', s
    )
    if not m:
        raise RuntimeError(f'Cannot read {model} pose:\n{s[-1000:]}')
    return (
        [float(v) for v in m.group(1).split()],
        [float(v) for v in m.group(2).split()]
    )

sdf = Path.home() / 'mrs_tray_ws/src/mrs_tray_demo/models/pick_block.sdf'
root = ET.parse(sdf).getroot()
collision = root.find('.//link/collision')
box = collision.find('./geometry/box/size') if collision is not None else None

if box is None or not box.text:
    raise RuntimeError('Box collision dimensions not found in SDF')

for element in (root.find('.//link'), collision):
    local_pose = element.find('pose') if element is not None else None
    if local_pose is not None and local_pose.text:
        if any(abs(float(v)) > 1e-5 for v in local_pose.text.split()):
            raise RuntimeError('Nonzero SDF geometry offset: inspect first')

size = [float(v) for v in box.text.split()]

(bx, by, bz), (_, _, block_yaw) = gz_pose('pick_block')
(ax, ay, az), (roll, pitch, arm_yaw) = gz_pose('lite6')

if abs(roll) > 0.01 or abs(pitch) > 0.01:
    raise RuntimeError('Unexpected arm-base tilt')

dx, dy = bx - ax, by - ay

x = math.cos(arm_yaw)*dx + math.sin(arm_yaw)*dy
y = -math.sin(arm_yaw)*dx + math.cos(arm_yaw)*dy
z = bz - az

print(f'Block in assumed link_base: {x:.4f}, {y:.4f}, {z:.4f}')
print('Block dimensions:', size)

obj = CollisionObject()
obj.id = 'orange_pick_block'
obj.header.frame_id = 'link_base'
obj.operation = CollisionObject.ADD

primitive = SolidPrimitive()
primitive.type = SolidPrimitive.BOX
primitive.dimensions = size
obj.primitives.append(primitive)

pose = Pose()
pose.position.x = x
pose.position.y = y
pose.position.z = z

relative_yaw = block_yaw - arm_yaw
pose.orientation.z = math.sin(relative_yaw/2)
pose.orientation.w = math.cos(relative_yaw/2)
obj.primitive_poses.append(pose)

color = ObjectColor()
color.id = obj.id
color.color.r = 1.0
color.color.g = 0.42
color.color.b = 0.04
color.color.a = 1.0

scene = PlanningScene()
scene.is_diff = True
scene.robot_state.is_diff = True
scene.world.collision_objects.append(obj)
scene.object_colors.append(color)


rclpy.init()
node = Node('tray_add_orange_block')

def apply_update(client, update, description):
    request = ApplyPlanningScene.Request()
    request.scene = update

    future = client.call_async(request)
    rclpy.spin_until_future_complete(
        node, future, timeout_sec=20.0
    )

    result = future.result() if future.done() else None

    if result is None or not result.success:
        raise RuntimeError(
            f"MoveIt planning-scene update failed: {description}"
        )

    print(f"SUCCESS: {description}")

try:
    client = node.create_client(
        ApplyPlanningScene, '/apply_planning_scene'
    )

    if not client.wait_for_service(timeout_sec=15):
        raise RuntimeError(
            "MoveIt /apply_planning_scene service unavailable"
        )

    # Add or replace block using its latest Gazebo pose.
    # 'scene' was built above with the correct geometry,
    # position, orientation, and orange color.
    apply_update(
        client, scene,
        "Added orange_pick_block at current Gazebo pose"
    )

    print(
        f"FINAL BLOCK POSITION (link_base): "
        f"x={x:.6f}, y={y:.6f}, z={z:.6f}"
    )

finally:
    node.destroy_node()
    rclpy.shutdown()
