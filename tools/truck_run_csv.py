#!/usr/bin/env python3
"""Send one dump-truck waypoint task and save localization data as CSV.

Run on the ROS PC after sourcing ROS, the workspace, and its Zenoh client.
Only the selected truck's odometry, wheel states, commands, tag TF, and
task feedback are saved. No rosbag process or extra recording terminal.
"""

import argparse
import csv
from datetime import datetime
import math
from pathlib import Path
import signal
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from construction_site_interfaces.action import ExecuteRobotTask
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage


FIELDS = (
    'receive_time_iso', 'elapsed_sec', 'source_stamp_sec', 'source',
    'x_m', 'y_m', 'yaw_deg', 'linear_x', 'angular_z',
    'left_ticks', 'right_ticks', 'detail',
)


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_degrees(quaternion):
    sin_yaw = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cos_yaw = 1 - 2 * (quaternion.y ** 2 + quaternion.z ** 2)
    return math.degrees(math.atan2(sin_yaw, cos_yaw))


class TruckRunCsv(Node):
    def __init__(self, args, writer, file_handle):
        super().__init__('truck_run_csv')
        self.args = args
        self.writer = writer
        self.file_handle = file_handle
        self.started = time.monotonic()
        self.rows = 0
        self.last_fused_received = None
        self.last_tag_received = None
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        prefix = '/' + args.robot
        self.create_subscription(Odometry, prefix + '/odom',
                                 lambda msg: self.on_odom('odom', msg), qos)
        self.create_subscription(Odometry, prefix + '/fused_odom',
                                 lambda msg: self.on_odom('fused_odom', msg), qos)
        self.create_subscription(JointState, prefix + '/wheel_states',
                                 self.on_wheels, qos)
        self.create_subscription(Twist, prefix + '/cmd_vel',
                                 self.on_command, qos)
        self.create_subscription(TFMessage, '/tf', self.on_tf, qos)
        self.client = ActionClient(self, ExecuteRobotTask,
                                   prefix + '/execute_robot_task')

    def row(self, source, stamp='', **kwargs):
        values = dict.fromkeys(FIELDS, '')
        values.update(kwargs)
        values.update(receive_time_iso=datetime.now().astimezone().isoformat(
            timespec='milliseconds'), elapsed_sec=round(time.monotonic() - self.started, 3),
            source_stamp_sec=stamp, source=source)
        self.writer.writerow(values)
        self.rows += 1
        if self.rows % 20 == 0:
            self.file_handle.flush()

    def on_odom(self, source, msg):
        self.row(source, stamp_seconds(msg.header.stamp),
                 x_m=msg.pose.pose.position.x, y_m=msg.pose.pose.position.y,
                 yaw_deg=yaw_degrees(msg.pose.pose.orientation),
                 linear_x=msg.twist.twist.linear.x,
                 angular_z=msg.twist.twist.angular.z)
        if source == 'fused_odom':
            self.last_fused_received = time.monotonic()

    def on_wheels(self, msg):
        self.row('wheel_states', stamp_seconds(msg.header.stamp),
                 left_ticks=msg.position[0] if len(msg.position) > 0 else '',
                 right_ticks=msg.position[1] if len(msg.position) > 1 else '')

    def on_command(self, msg):
        self.row('cmd_vel', linear_x=msg.linear.x, angular_z=msg.angular.z)

    def on_tf(self, msg):
        for transform in msg.transforms:
            if (transform.header.frame_id != self.args.camera_frame or
                    transform.child_frame_id != self.args.tag_frame):
                continue
            self.last_tag_received = time.monotonic()
            self.row('tag_tf_camera', stamp_seconds(transform.header.stamp),
                     x_m=transform.transform.translation.x,
                     y_m=transform.transform.translation.y,
                     yaw_deg=yaw_degrees(transform.transform.rotation),
                     detail=self.args.tag_frame)

    def on_feedback(self, message):
        feedback = message.feedback
        self.row('action_feedback', detail=(
            f'{feedback.state}: {feedback.detail}; progress={feedback.progress:.3f}'))


def spin_until(node, condition, deadline, interrupted):
    while rclpy.ok() and not condition():
        if interrupted() or time.monotonic() >= deadline:
            return False
        rclpy.spin_once(node, timeout_sec=0.1)
    return bool(condition())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('task_file', type=Path)
    parser.add_argument('--robot', default='truck1')
    parser.add_argument('--tag-frame', default='tag36h11_0')
    parser.add_argument('--camera-frame', default='default_cam')
    parser.add_argument('--label', default='run', help='Name in CSV filename')
    parser.add_argument('--timeout', type=float, default=90.0,
                        help='Maximum task duration, seconds')
    parser.add_argument('--output', type=Path, default=None)
    args = parser.parse_args()
    args.task_file = args.task_file.expanduser().resolve()
    if not args.task_file.is_file():
        parser.error(f'task file does not exist: {args.task_file}')
    if args.timeout <= 0:
        parser.error('--timeout must be positive')
    if args.output is None:
        filename = f'{args.robot}_{args.label}_{datetime.now():%Y%m%d_%H%M%S}.csv'
        args.output = Path.home() / 'truck_test_logs' / filename
    args.output = args.output.expanduser().resolve()
    args.output.parent.mkdir(parents=True, exist_ok=True)

    interrupted = False

    def on_interrupt(_signum, _frame):
        nonlocal interrupted
        interrupted = True

    rclpy.init()
    signal.signal(signal.SIGINT, on_interrupt)
    node = None
    goal_handle = None
    status = 1
    try:
        with args.output.open('w', newline='', encoding='utf-8') as file_handle:
            writer = csv.DictWriter(file_handle, fieldnames=FIELDS)
            writer.writeheader()
            node = TruckRunCsv(args, writer, file_handle)
            print(f'CSV: {args.output}', flush=True)
            print('Waiting for fresh fused odometry and robot AprilTag TF...', flush=True)
            ready = spin_until(
                node,
                lambda: (node.last_fused_received is not None and
                         node.last_tag_received is not None and
                         time.monotonic() - node.last_fused_received < 1.0 and
                         time.monotonic() - node.last_tag_received < 1.0),
                time.monotonic() + 10.0, lambda: interrupted)
            if not ready:
                print('No fresh fused odometry/tag within 10 seconds; no goal sent.',
                      file=sys.stderr)
                return status
            if not node.client.wait_for_server(timeout_sec=5.0):
                print('Truck Action server unavailable; no goal sent.', file=sys.stderr)
                return status
            if interrupted:
                print('Interrupted before sending goal.', file=sys.stderr)
                return status
            rclpy.spin_once(node, timeout_sec=0.1)
            if (node.last_fused_received is None or node.last_tag_received is None or
                    time.monotonic() - node.last_fused_received >= 1.0 or
                    time.monotonic() - node.last_tag_received >= 1.0):
                print('Feedback or robot tag went stale; no goal sent.',
                      file=sys.stderr)
                return status
            request = ExecuteRobotTask.Goal()
            request.robot_name = args.robot
            request.task_type = 'waypoint'
            request.task_file = str(args.task_file)
            print(f'Sending task: {args.task_file}', flush=True)
            sent = node.client.send_goal_async(request, feedback_callback=node.on_feedback)
            if not spin_until(node, sent.done, time.monotonic() + 10.0,
                              lambda: interrupted):
                print('No goal response within 10 seconds.', file=sys.stderr)
                return status
            goal_handle = sent.result()
            if not goal_handle.accepted:
                print('Goal rejected.', file=sys.stderr)
                return status
            print('Goal accepted; recording until the task finishes.', flush=True)
            completed = goal_handle.get_result_async()
            deadline = time.monotonic() + args.timeout
            if not spin_until(node, completed.done, deadline, lambda: interrupted):
                print('Interrupted or timed out; requesting goal cancellation.',
                      file=sys.stderr)
                canceled = goal_handle.cancel_goal_async()
                spin_until(node, canceled.done, time.monotonic() + 3.0,
                           lambda: False)
                return status
            result = completed.result().result
            node.row('action_result', detail=(
                f'success={result.success}; {result.message}'))
            print(f'Action result: success={result.success}; {result.message}',
                  flush=True)
            # Capture the pose after the server has commanded a stop.
            spin_until(node, lambda: False, time.monotonic() + 1.0,
                       lambda: interrupted)
            status = 0 if result.success else 1
    finally:
        if node is not None:
            print(f'Saved {node.rows} CSV rows: {args.output}', flush=True)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return status


if __name__ == '__main__':
    sys.exit(main())
