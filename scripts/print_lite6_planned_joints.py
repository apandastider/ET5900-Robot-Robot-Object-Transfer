#!/usr/bin/env python3
"""Print last joint positions from a MoveIt DisplayTrajectory plan.
Run before clicking 'Plan' in RViz. Does not send robot motion commands.
"""
import sys
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import DisplayTrajectory


class LastPlan(Node):
    def __init__(self, topic):
        super().__init__('mrs_print_lite6_planned_joints')
        self.create_subscription(DisplayTrajectory, topic, self.callback, 10)
        self.get_logger().info('Waiting for a planned path on ' + topic + '; click Plan in RViz (not Execute).')

    def callback(self, msg):
        for segment in reversed(msg.trajectory):
            trajectory = segment.joint_trajectory
            if not trajectory.joint_names or not trajectory.points:
                continue
            point = trajectory.points[-1]
            print('\nLast waypoint of latest planned trajectory (radians):')
            for name, pos in zip(trajectory.joint_names, point.positions):
                if name in {f'joint{i}' for i in range(1, 7)}:
                    print(f'  {name}: {pos:+.6f}')
            print('This is a planned joint target, not an executed joint position.\n')
            sys.stdout.flush()
            return
        self.get_logger().warning('No joint trajectory points in this message.')


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else '/display_planned_path'
    rclpy.init()
    node = LastPlan(topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
