#!/usr/bin/env python3
"""Simple autonomous TurtleBot4 + Lite6 Gazebo tray-transfer FSM.

ROS 2 Jazzy, Gazebo Sim 8. Uses existing MoveIt2 planning service,
Gazebo's lite6_traj_controller FollowJointTrajectory action, Nav2,
and the ALREADY TESTED ~/mrs_tray_ws/scripts/virtual_grasp_hold.py.

Only for simulation. Does not launch Gazebo or manipulate hardware.
"""
import argparse
import copy
import math
import queue
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionClient, get_action_names_and_types
from rclpy.node import Node
from rclpy.parameter import Parameter
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import (Constraints, JointConstraint, CollisionObject,
                           PlanningScene, PlanningSceneComponents)
from moveit_msgs.srv import GetMotionPlan, ApplyPlanningScene, GetPlanningScene
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import JointState
from std_msgs.msg import String

JOINT_NAMES = [f'joint{i}' for i in range(1, 7)]
POSES = {
    'pregrasp': [1.2829, 1.4382, 2.1537, 0.0, 0.7154, 1.2828],
    'lift':     [0.9561, 0.4754, 1.5481, 0.0, 1.0727, 0.9560],
    'place':    [-0.4864, 0.5401, 0.6637, -0.0001, 0.1236, -0.4862],
}
HOME = (-3.00, -1.60, 0.0)
TRANSFER = (-0.40, 0.15, 0.0)
DELIVERY = (-2.30, 1.45, 0.0)
WORLD = 'milestone2_multi_robot_transfer'
HOLD_SCRIPT = Path.home() / 'mrs_tray_ws/scripts/virtual_grasp_hold.py'
ORANGE_SCENE_IDS = {'orange_pick_block', 'orange_block', 'pick_block'}


def check_gz_model(model):
    try:
        r = subprocess.run(['gz', 'model', '-m', model, '--pose'],
                           capture_output=True, text=True, timeout=18)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise RuntimeError(f'Gazebo check failed for {model}: {exc}') from exc
    msg = (r.stdout + '\n' + r.stderr).strip()
    if r.returncode != 0 or 'Pose' not in msg:
        raise RuntimeError(f'Gazebo model {model!r} not found; is the right world running? {msg[-450:]}')
    return msg


def check_gz_scene_models(models):
    """Verify models with ONE world-specific call, avoiding /gazebo/worlds lookup.

    `gz model` first queries /gazebo/worlds each invocation. That can time
    out intermittently under a busy multi-robot simulation even when all
    models exist. /world/<known world>/scene/info avoids that discovery step.
    """
    service = f'/world/{WORLD}/scene/info'
    cmd = ['gz', 'service', '-s', service,
           '--reqtype', 'gz.msgs.Empty', '--reptype', 'gz.msgs.Scene',
           '--timeout', '8000', '--req', '']
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=14)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise RuntimeError(f'Cannot query Gazebo scene via {service}: {exc}. '
                           'Ensure the Gazebo server is responsive; do not assume models are missing.') from exc
    out = result.stdout + '\n' + result.stderr
    if result.returncode != 0 or not result.stdout.strip():
        raise RuntimeError(f'Gazebo scene query failed for {service}: {out[-450:]}. '
                           'This is a service failure, not proof that the TurtleBot is absent.')
    missing = [model for model in models if f'name: "{model}"' not in result.stdout]
    if missing:
        raise RuntimeError(f'Gazebo scene responded, but models {missing} were not present. '
                           f'Check model names in {service}.')
    for model in models:
        print(f'  Gazebo: {model} detected (scene/info)', flush=True)


class TransferFSM(Node):
    def __init__(self, args):
        super().__init__('tray_transfer_fsm')
        # ROS 2 Jazzy declares use_sim_time on many nodes automatically.
        # Set the existing parameter instead of redeclaring it.
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.args = args
        self.hold = None
        self.hold_queue = queue.Queue()
        self.hold_reader = None
        self.state_pub = self.create_publisher(String, '/tray_transfer_fsm/state', 10)
        self.latest_joints = {}
        self.create_subscription(JointState, '/lite6/joint_states', self.on_joints, 10)
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.plan_client = None
        self.arm_client = None

    def on_joints(self, msg):
        self.latest_joints.update({n: p for n, p in zip(msg.name, msg.position)})

    def state(self, state, message=''):
        line = f'[FSM: {state}] {message}'.strip()
        print('\n' + line, flush=True)
        out = String()
        out.data = line
        self.state_pub.publish(out)

    def until(self, condition, timeout, what):
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            if condition():
                return
            rclpy.spin_once(self, timeout_sec=0.15)
        raise TimeoutError(f'Timed out waiting for {what} ({timeout:.0f} s)')

    def future_result(self, future, timeout, what):
        self.until(lambda: future.done(), timeout, what)
        result = future.result()
        if result is None:
            raise RuntimeError(f'{what} returned no result')
        return result

    def locate_arm_action(self):
        # Avoid silently commanding a physical or unrelated controller.
        if self.args.controller_action:
            return self.args.controller_action
        matches = []
        for name, types in get_action_names_and_types(self):
            if (name.endswith('/lite6_traj_controller/follow_joint_trajectory') and
                    'control_msgs/action/FollowJointTrajectory' in types):
                matches.append(name)
        if len(matches) != 1:
            raise RuntimeError('Expected exactly one Lite6 Gazebo trajectory action. '
                               f'Found {matches}. Run: ros2 action list -t; '
                               'or pass --controller-action /.../follow_joint_trajectory')
        return matches[0]

    def locate_planner(self):
        services = [(n, types) for n, types in self.get_service_names_and_types()
                    if n.endswith('/plan_kinematic_path') and
                    'moveit_msgs/srv/GetMotionPlan' in types]
        if len(services) != 1:
            raise RuntimeError(f'Expected one MoveIt planning service, found {services}. '
                               'Run: ros2 service list -t | grep plan_kinematic_path')
        return services[0][0]

    def check_ready(self):
        self.state('CHECK', 'Checking Gazebo, ROS 2 actions and MoveIt planner')
        check_gz_scene_models(('lite6', 'pick_block', 'turtlebot4'))
        if not self.nav_client.wait_for_server(timeout_sec=30.0):
            raise RuntimeError('Nav2 /navigate_to_pose action is not ready')
        self.until(lambda: all(j in self.latest_joints for j in JOINT_NAMES),
                   15, '/lite6/joint_states')
        # Let DDS discover services/actions if launches are still starting.
        arm_name = None
        plan_name = None
        until = time.monotonic() + 20
        while time.monotonic() < until and (arm_name is None or plan_name is None):
            rclpy.spin_once(self, timeout_sec=0.2)
            try:
                arm_name = self.locate_arm_action()
            except RuntimeError:
                pass
            try:
                plan_name = self.locate_planner()
            except RuntimeError:
                pass
        if not arm_name:
            raise RuntimeError('Lite6 trajectory controller action not found. Use ros2 action list -t')
        if not plan_name:
            raise RuntimeError('MoveIt plan-only service not found. Start lite6_moveit_tray_only.launch.py')
        self.arm_client = ActionClient(self, FollowJointTrajectory, arm_name)
        self.plan_client = self.create_client(GetMotionPlan, plan_name)
        if not self.arm_client.wait_for_server(timeout_sec=12):
            raise RuntimeError(f'Controller {arm_name} did not become ready')
        if not self.plan_client.wait_for_service(timeout_sec=12):
            raise RuntimeError(f'MoveIt service {plan_name} did not become ready')
        # Nav2 requires a valid saved map and localization. The action server
        # existing is not proof localization is good; it will reject failed goals.
        print(f'  Navigation: /navigate_to_pose\n  MoveIt: {plan_name}\n  Arm: {arm_name}', flush=True)
        if not HOLD_SCRIPT.exists():
            raise RuntimeError(f'Missing original virtual grasp script: {HOLD_SCRIPT}')
        # The SetEntityPose service must be bridged separately, not gz topic attach.
        srv = self.create_client(
            __import__('ros_gz_interfaces.srv', fromlist=['SetEntityPose']).SetEntityPose,
            f'/world/{WORLD}/set_pose')
        if not srv.wait_for_service(timeout_sec=12):
            raise RuntimeError(f'Missing /world/{WORLD}/set_pose service bridge')
        self.destroy_client(srv)

    def navigate(self, target, name):
        x, y, yaw = target
        self.state(name, f'Nav2 to ({x:+.2f}, {y:+.2f}), yaw={yaw:.2f} rad')
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        future = self.nav_client.send_goal_async(goal)
        handle = self.future_result(future, 15, f'{name} goal acceptance')
        if not handle.accepted:
            raise RuntimeError(f'Nav2 rejected goal for {name}')
        response = self.future_result(handle.get_result_async(), self.args.nav_timeout,
                                      f'{name} navigation completion')
        if response.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f'Nav2 failed during {name}: status={response.status}, '
                               f'error={getattr(response.result, "error_code", "unknown")}')
        if getattr(response.result, 'error_code', 0) != 0:
            raise RuntimeError(f'Nav2 reported error_code={response.result.error_code}')
        self.state(name, 'Navigation SUCCEEDED')
        time.sleep(1.0)

    def plan_arm(self, name, target):
        self.state(name, 'Planning with MoveIt 2 (collision-aware for configured scene)')
        req = GetMotionPlan.Request()
        m = req.motion_plan_request
        m.group_name = 'lite6'
        m.start_state.is_diff = True  # Take current robot state from MoveIt.
        m.num_planning_attempts = 8
        m.allowed_planning_time = 12.0
        m.max_velocity_scaling_factor = self.args.speed
        m.max_acceleration_scaling_factor = self.args.speed
        c = Constraints()
        c.name = name
        for n, value in zip(JOINT_NAMES, target):
            j = JointConstraint()
            j.joint_name = n
            j.position = float(value)
            j.tolerance_above = 0.015
            j.tolerance_below = 0.015
            j.weight = 1.0
            c.joint_constraints.append(j)
        m.goal_constraints = [c]
        result = self.future_result(self.plan_client.call_async(req), 25,
                                    f'MoveIt plan for {name}')
        response = result.motion_plan_response
        if response.error_code.val != 1:
            raise RuntimeError(f'MoveIt planning FAILED for {name}: code={response.error_code.val}. '
                               'Inspect MoveIt planning scene and trajectory in RViz.')
        jt = response.trajectory.joint_trajectory
        if not jt.points or set(jt.joint_names) != set(JOINT_NAMES):
            raise RuntimeError(f'Invalid MoveIt joint trajectory for {name}: {jt.joint_names}')
        planned = {n: value for n, value in zip(jt.joint_names, jt.points[-1].positions)}
        if any(abs(planned[n] - value) > 0.10 for n, value in zip(JOINT_NAMES, target)):
            raise RuntimeError(f'MoveIt planned endpoint does not match {name} target')
        return copy.deepcopy(jt)

    def move_arm(self, name, target):
        jt = self.plan_arm(name, target)
        self.state(name, f'Executing MoveIt trajectory in Gazebo ({len(jt.points)} points)')
        jt.header.stamp.sec = 0
        jt.header.stamp.nanosec = 0  # Start when the controller receives it.
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = jt
        future = self.arm_client.send_goal_async(goal)
        handle = self.future_result(future, 15, f'{name} trajectory acceptance')
        if not handle.accepted:
            raise RuntimeError(f'Lite6 controller rejected trajectory {name}')
        duration = jt.points[-1].time_from_start.sec + jt.points[-1].time_from_start.nanosec * 1e-9
        result = self.future_result(handle.get_result_async(), max(35.0, duration + 25.0),
                                    f'{name} arm trajectory completion')
        if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code != 0:
            raise RuntimeError(f'Lite6 trajectory failed: {name}, status={result.status}, '
                               f'error_code={result.result.error_code}, '
                               f'error_string={result.result.error_string}')
        # Verify actual Gazebo joint state rather than only trusting the action result.
        self.until(lambda: (all(n in self.latest_joints for n in JOINT_NAMES) and
                            all(abs(self.latest_joints[n] - v) < 0.10
                                for n, v in zip(JOINT_NAMES, target))),
                   6.0, f'Lite6 reaching {name} pose within 0.10 rad')
        self.state(name, 'Arm reached target (joint feedback verified)')

    def moveit_world_ids(self):
        """Get world collision-object IDs from the *same* MoveIt planning scene.

        Unlike Gazebo's ``pick_block`` model, a MoveIt collision object is
        optional and may have been added with a different ID.
        """
        matches = [name for name, types in self.get_service_names_and_types()
                   if name.endswith('/get_planning_scene') and
                   'moveit_msgs/srv/GetPlanningScene' in types]
        if len(matches) != 1:
            raise RuntimeError(f'Expected one MoveIt get_planning_scene service; got {matches}. '
                               'Check: ros2 service list -t | grep get_planning_scene')
        client = self.create_client(GetPlanningScene, matches[0])
        try:
            if not client.wait_for_service(timeout_sec=8):
                raise RuntimeError('MoveIt get_planning_scene service not ready')
            req = GetPlanningScene.Request()
            req.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
            result = self.future_result(client.call_async(req), 12,
                                        'Read MoveIt collision-object names')
            return {obj.id for obj in result.scene.world.collision_objects}
        finally:
            self.destroy_client(client)

    def clear_orange_moveit_obstacle(self):
        """Remove only existing static orange-block collision objects from MoveIt.

        Gazebo's pick_block model is completely separate and stays in Gazebo.
        A REMOVE request for an absent MoveIt collision object returns false.
        That is an already-satisfied removal, not an error in the grasp.
        """
        world_ids = self.moveit_world_ids()
        print(f'  MoveIt world collision objects: {sorted(world_ids)}', flush=True)
        orange_ids = sorted(world_ids.intersection(ORANGE_SCENE_IDS))
        if not orange_ids:
            print('  No orange block collision object is present in MoveIt; '
                  'nothing to remove. Gazebo pick_block stays attached virtually.', flush=True)
            return

        matches = [name for name, types in self.get_service_names_and_types()
                   if name.endswith('/apply_planning_scene') and
                   'moveit_msgs/srv/ApplyPlanningScene' in types]
        if len(matches) != 1:
            raise RuntimeError(f'Expected one MoveIt apply_planning_scene service; got {matches}')
        client = self.create_client(ApplyPlanningScene, matches[0])
        try:
            if not client.wait_for_service(timeout_sec=8):
                raise RuntimeError('MoveIt apply_planning_scene service not ready')
            req = ApplyPlanningScene.Request()
            req.scene = PlanningScene()
            req.scene.is_diff = True
            # No need to specify header.frame_id for a REMOVE operation.
            for object_id in orange_ids:
                obj = CollisionObject()
                obj.id = object_id
                obj.operation = CollisionObject.REMOVE
                req.scene.world.collision_objects.append(obj)
            res = self.future_result(client.call_async(req), 12,
                                     'Remove existing MoveIt orange collision objects')
        finally:
            self.destroy_client(client)

        # Confirm the result from MoveIt's actual scene, not just success=True.
        remaining = set(orange_ids).intersection(self.moveit_world_ids())
        if remaining:
            raise RuntimeError(f'MoveIt still contains stale orange obstacles: '
                               f'{sorted(remaining)}; apply success={res.success}')
        print('  MoveIt orange static collision object(s) cleared: '
              f'{orange_ids}; Gazebo pick_block remains present', flush=True)

    def _read_hold_output(self):
        for line in self.hold.stdout:
            line = line.rstrip()
            print('  [grasp] ' + line, flush=True)
            self.hold_queue.put(line)

    def attach(self):
        self.state('ATTACH', 'Starting verified virtual grasp (Gazebo SetEntityPose)')
        if self.hold is not None:
            raise RuntimeError('Grasp subprocess already running')
        self.hold = subprocess.Popen(
            [sys.executable, '-u', str(HOLD_SCRIPT), '--world', WORLD,
             '--block', 'pick_block', '--arm', 'lite6', '--rate', '15',
             '--max-gap', str(self.args.grasp_max_gap)],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1)
        self.hold_reader = threading.Thread(target=self._read_hold_output, daemon=True)
        self.hold_reader.start()
        confirmed = False
        timeout = time.monotonic() + 45
        while time.monotonic() < timeout:
            if self.hold.poll() is not None:
                raise RuntimeError('Virtual grasp helper stopped unexpectedly; check [grasp] logs')
            try:
                line = self.hold_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if 'First Gazebo pose update CONFIRMED.' in line:
                confirmed = True
                break
        if not confirmed:
            raise RuntimeError('Virtual grasp did not confirm a Gazebo pose update')
        self.clear_orange_moveit_obstacle()
        self.state('ATTACH', 'Virtual grasp ACTIVE and first update confirmed')

    def check_hold(self):
        if self.hold is None or self.hold.poll() is not None:
            raise RuntimeError('Virtual grasp died while the robot was holding the block!')

    def detach(self):
        self.state('RELEASE', 'Stopping virtual grasp; block lands in receiving area if supported by physics')
        if self.hold is None:
            raise RuntimeError('No active grasp to release')
        if self.hold.poll() is None:
            self.hold.send_signal(signal.SIGINT)
        try:
            self.hold.wait(timeout=12)
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError('Virtual grasp process did not release promptly; halted demo') from exc
        self.hold = None
        time.sleep(2.0)
        try:
            print('  Gazebo block pose after release:', check_gz_model('pick_block').splitlines()[-1], flush=True)
        except RuntimeError as exc:
            print(f'  WARNING: Could not query block pose after release: {exc}', flush=True)
            print('  Check the final block location visually in Gazebo.', flush=True)

    def cleanup(self):
        if self.hold is not None and self.hold.poll() is None:
            print('  Stopping virtual grasp helper during cleanup', flush=True)
            self.hold.send_signal(signal.SIGINT)
            try:
                self.hold.wait(timeout=6)
            except subprocess.TimeoutExpired:
                self.hold.kill()
                self.hold.wait(timeout=2)

    def run_demo(self):
        self.check_ready()
        if self.args.check_only:
            self.state('READY', 'Checks passed. No motion commanded.')
            return
        self.navigate(TRANSFER, 'NAVIGATE_TO_TRANSFER')
        self.move_arm('PREGRASP', POSES['pregrasp'])
        self.attach()
        self.check_hold()
        self.move_arm('LIFT', POSES['lift'])
        self.check_hold()
        self.move_arm('PLACE', POSES['place'])
        self.check_hold()
        self.detach()
        if not self.args.skip_retract:
            self.move_arm('RETRACT', POSES['lift'])
        self.navigate(DELIVERY, 'NAVIGATE_TO_DELIVERY')
        self.state('DONE', 'TurtleBot reached final goal. Verify the orange block was deposited at the receiving area.')
        print('\nDEMO COMPLETE — verify block in receiving area and TurtleBot at delivery goal.', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check-only', action='store_true', help='Read-only checks; no movement')
    parser.add_argument('--controller-action', default='/lite6/lite6_traj_controller/follow_joint_trajectory', help='Lite6 Gazebo controller action (can override)')
    parser.add_argument('--nav-timeout', type=float, default=240.0)
    parser.add_argument('--speed', type=float, default=0.15,
                        help='MoveIt maximum velocity/acceleration scale (0..1)')
    parser.add_argument('--grasp-max-gap', type=float, default=0.16,
                        help='Maximum pregrasp tool-to-block distance (m)')
    parser.add_argument('--skip-retract', action='store_true',
                        help='For debugging only; robot may depart while arm is over tray')
    args = parser.parse_args()
    if not 0 < args.speed <= 1:
        parser.error('--speed must be in (0,1]')
    rclpy.init()
    node = TransferFSM(args)
    try:
        node.run_demo()
    except KeyboardInterrupt:
        print('\nFSM interrupted (Ctrl+C): aborting further steps', flush=True)
        sys.exit(130)
    except Exception as exc:
        node.state('FAILED', str(exc))
        print('Demo stopped safely: later FSM transitions were NOT executed.', flush=True)
        sys.exit(1)
    finally:
        node.cleanup()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
