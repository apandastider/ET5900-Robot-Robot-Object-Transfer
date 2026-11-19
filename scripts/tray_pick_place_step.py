#!/usr/bin/env python3
import argparse
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from builtin_interfaces.msg import Time
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.msg import DisplayTrajectory, CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from sensor_msgs.msg import JointState

class Demo(Node):
    def __init__(self):
        super().__init__('mrs_lite6_one_step')
        self.path, self.actual = None, None
        self.create_subscription(
            DisplayTrajectory, '/display_planned_path', self.on_plan, 10)
        self.create_subscription(
            JointState, '/lite6/joint_states', self.on_state,
            qos_profile_sensor_data)

    def on_state(self, m):
        self.actual = dict(zip(m.name, m.position))

    def on_plan(self, m):
        valid = [t.joint_trajectory for t in m.trajectory
                 if t.joint_trajectory.points]
        expected = {f'joint{i}' for i in range(1, 7)}
        if (self.path is None and len(valid) == 1
                and set(valid[0].joint_names) == expected):
            self.path = valid[0]

def await_condition(n, condition, seconds):
    end = time.monotonic() + seconds
    while rclpy.ok() and not condition() and time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.2)
    if not condition():
        raise RuntimeError('Timeout: expected ROS data not received')

def remove_block(n):
    c = n.create_client(ApplyPlanningScene, '/apply_planning_scene')
    if not c.wait_for_service(timeout_sec=10):
        raise RuntimeError('Missing /apply_planning_scene service')

    obj = CollisionObject(id='orange_pick_block')
    obj.header.frame_id = 'link_base'
    obj.operation = CollisionObject.REMOVE

    scene = PlanningScene(is_diff=True)
    scene.robot_state.is_diff = True
    scene.world.collision_objects = [obj]

    f = c.call_async(ApplyPlanningScene.Request(scene=scene))
    rclpy.spin_until_future_complete(n, f, timeout_sec=10)

    if not f.done() or f.result() is None or not f.result().success:
        raise RuntimeError('Could not update planning scene')
    print('Removed block from MoveIt only; Gazebo block remains.')

def execute_plan(n):
    print('Click Plan in RViz for the next safe target.')
    await_condition(n, lambda: n.path is not None, 180)
    await_condition(n, lambda: n.actual is not None, 10)

    names, points = n.path.joint_names, n.path.points

    if not points or any(len(p.positions) != 6 for p in points):
        raise RuntimeError('Invalid six-joint trajectory')

    if any(j not in n.actual for j in names):
        raise RuntimeError('Missing arm joint measurements')

    deviation = max(
        abs(n.actual[j] - v)
        for j, v in zip(names, points[0].positions)
    )

    if deviation > 0.15:
        raise RuntimeError(
            f'Stale start state ({deviation:.3f} rad mismatch). '
            'Replan from Current state.'
        )

    print('\nPlanned final joint angles:')
    for j, p in zip(names, points[-1].positions):
        if not math.isfinite(p):
            raise RuntimeError('Invalid joint angle')
        print(f'  {j}: {p:+.4f} rad ({math.degrees(p):+.1f} deg)')

    if input('Type EXECUTE to move the Gazebo arm: ').strip() != 'EXECUTE':
        print('Cancelled.')
        return

    client = ActionClient(
        n, FollowJointTrajectory,
        '/lite6/lite6_traj_controller/follow_joint_trajectory'
    )

    if not client.wait_for_server(timeout_sec=10):
        raise RuntimeError('Lite 6 arm controller unavailable')

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = n.path
    goal.trajectory.header.stamp = Time()
    goal.goal_time_tolerance.sec = 3

    f = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(n, f, timeout_sec=10)

    if not f.done() or f.result() is None or not f.result().accepted:
        raise RuntimeError('Controller rejected trajectory')

    result = f.result().get_result_async()
    rclpy.spin_until_future_complete(n, result, timeout_sec=180)

    if (not result.done() or result.result() is None
            or result.result().result.error_code != 0):
        raise RuntimeError('Controller did not report successful execution')

    print('SUCCESS: Lite 6 moved in Gazebo.')

if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--remove-block', action='store_true')
    a = p.parse_args()

    rclpy.init()
    n = Demo()
    try:
        remove_block(n) if a.remove_block else execute_plan(n)
    finally:
        n.destroy_node()
        rclpy.shutdown()
