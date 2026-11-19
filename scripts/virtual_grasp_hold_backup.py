#!/usr/bin/env python3
"""Demonstration-only virtual grasp for Gazebo Sim / ROS 2 Jazzy.

Keeps Gazebo model 'pick_block' approximately aligned with Lite 6 'link6'
by repeatedly calling the standard /world/<world>/set_pose service.

Not a physical grasp, not a rigid constraint, and not a MoveIt collision-object
attachment. For controlled demonstration only; do not use as contact physics.
Start when link6 is close to the block; Ctrl+C to release.
"""
import argparse
import math
import re
import subprocess
import time
from typing import Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

Pose = Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]


def qmul(a, b):
    w, x, y, z = a
    v, i, j, k = b
    return (w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j,
            w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v)


def qconj(a):
    return (a[0], -a[1], -a[2], -a[3])


def qnorm(q):
    l = math.sqrt(sum(x*x for x in q))
    return tuple(x/l for x in q)


def qrotate(q, v):
    return qmul(qmul(q, (0.0, *v)), qconj(q))[1:]


def compose(a: Pose, b: Pose) -> Pose:
    p, q = a
    pb, qb = b
    r = qrotate(q, pb)
    return ((p[0]+r[0], p[1]+r[1], p[2]+r[2]), qnorm(qmul(q, qb)))


def inverse(a: Pose) -> Pose:
    p, q = a
    qi = qconj(q)
    return (qrotate(qi, (-p[0], -p[1], -p[2])), qi)


def quat_from_rpy(r, p, y):
    cr, sr = math.cos(r/2), math.sin(r/2)
    cp, sp = math.cos(p/2), math.sin(p/2)
    cy, sy = math.cos(y/2), math.sin(y/2)
    return (cr*cp*cy + sr*sp*sy,
            sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy)


def gz_model_pose(model: str) -> Pose:
    """Query Gazebo pose with retries."""
    attempts = []

    for n in range(5):
        try:
            proc = subprocess.run(
                ['gz', 'model', '-m', model, '-p'],
                capture_output=True,
                text=True,
                timeout=20
            )

            output = proc.stdout + '\n' + proc.stderr

            m = re.search(
                r'Pose \[ XYZ \(m\) \] \[ RPY \(rad\) \]:\s*'
                r'\[([^]]+)\]\s*\[([^]]+)\]',
                output
            )

            if m:
                xyz = tuple(map(float, m.group(1).split()))
                r, pitch, yaw = map(float, m.group(2).split())
                return xyz, quat_from_rpy(r, pitch, yaw)

            attempts.append(
                f'#{n+1}: rc={proc.returncode}, output={output[-250:]!r}'
            )

        except subprocess.TimeoutExpired:
            attempts.append(f'#{n+1}: gz model timed out')

        time.sleep(1)

    raise RuntimeError(
        'Cannot read Gazebo pose for ' + model
        + '\n' + '\n'.join(attempts)
    )


def pose_from_tf(transform) -> Pose:
    t = transform.transform.translation
    r = transform.transform.rotation
    return ((t.x, t.y, t.z), (r.w, r.x, r.y, r.z))


def set_pose(world: str, model: str, pose: Pose) -> bool:
    p, q = pose
    req = (f'name: "{model}" position: {{x: {p[0]:.7f}, y: {p[1]:.7f}, z: {p[2]:.7f}}} '
           f'orientation: {{w: {q[0]:.8f}, x: {q[1]:.8f}, y: {q[2]:.8f}, z: {q[3]:.8f}}}')
    result = subprocess.run([
        'gz', 'service', '-s', f'/world/{world}/set_pose',
        '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
        '--timeout', '5000', '--req', req
    ], capture_output=True, text=True, timeout=10)
    ok = result.returncode == 0 and 'data: true' in result.stdout.lower()
    if not ok:
        print('Gazebo service output:', result.stdout, result.stderr, flush=True)
    return ok


def main():
    ap = argparse.ArgumentParser(description='Virtual pickup: hold Gazebo block near arm tool until Ctrl+C')
    ap.add_argument('--world', default='milestone2_multi_robot_transfer')
    ap.add_argument('--block', default='pick_block')
    ap.add_argument('--arm', default='lite6')
    ap.add_argument('--base-frame', default='link_base')
    ap.add_argument('--tool-frame', default='link6')
    ap.add_argument('--max-gap', type=float, default=0.16,
                    help='Maximum tool-to-block-center distance (m) at pickup')
    ap.add_argument('--rate', type=float, default=8.0, help='Pose-update frequency, Hz')
    args = ap.parse_args()
    if args.rate <= 0:
        ap.error('--rate must be positive')

    rclpy.init()
    node = Node('lite6_virtual_grasp_hold')
    buf = Buffer()
    listener = TransformListener(buf, node)
    try:
        arm_world = gz_model_pose(args.arm)
        block_world = gz_model_pose(args.block)
        tf = None
        timeout_at = time.monotonic() + 12
        while time.monotonic() < timeout_at:
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                tf = buf.lookup_transform(args.base_frame, args.tool_frame, Time())
                break
            except Exception:
                pass
        if tf is None:
            raise RuntimeError(f'No TF {args.base_frame} <- {args.tool_frame}; check /tf and robot_state_publisher')

        tool_world = compose(arm_world, pose_from_tf(tf))
        gap = math.dist(tool_world[0], block_world[0])
        print(f'Tool-to-block distance: {gap:.3f} m')
        if gap > args.max_gap:
            raise RuntimeError('Move Lite 6 to its pre-grasp pose first; object is too far away. No attachment started.')
        relative = compose(inverse(tool_world), block_world)
        print(f'VIRTUAL ATTACH ACTIVE: {args.block} follows {args.tool_frame} ({args.rate:.1f} Hz).')
        print('Execute your approved Lite 6 arm trajectory in another terminal.')
        print('Move to a supported place pose BEFORE pressing Ctrl+C to release. Do not close the gripper.')

        errors = 0
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            try:
                tf = buf.lookup_transform(args.base_frame, args.tool_frame, Time())
                new_tool_world = compose(arm_world, pose_from_tf(tf))
                new_block_world = compose(new_tool_world, relative)
                if not set_pose(args.world, args.block, new_block_world):
                    errors += 1
                    if errors in (1, 5):
                        print('WARNING: Gazebo set_pose did not confirm success. Check service/model name.', flush=True)
                else:
                    errors = 0
            except Exception as e:
                errors += 1
                if errors in (1, 5):
                    print(f'WARNING: {e}', flush=True)
            time.sleep(1.0 / args.rate)
    except KeyboardInterrupt:
        print('\nVIRTUAL ATTACH RELEASED. Block is dynamic in Gazebo again.')
    finally:
        del listener
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
