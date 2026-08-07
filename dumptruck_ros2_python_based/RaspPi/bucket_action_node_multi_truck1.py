#!/usr/bin/env python3
"""
bucket_action_node_multi.py

Bucket/dump servo node for one dump truck.

Input:
  /truckN/bucket_action_cmd

Output:
  /truckN/bucket_action_status
"""

import time
import pigpio
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

SERVO_PIN = 10
SERVO_CENTER = 900
SERVO_DUMP = 1300
MOVE_DURATION = 2.0
HOLD_TIME = 2.0
STEPS = 100


class BucketActionNode(Node):
    def __init__(self):
        super().__init__('bucket_action_node')

        self.declare_parameter('bucket_action_cmd_topic', '/bucket_action_cmd')
        self.declare_parameter('bucket_action_status_topic', '/bucket_action_status')

        self.cmd_topic = str(self.get_parameter('bucket_action_cmd_topic').value)
        self.status_topic = str(self.get_parameter('bucket_action_status_topic').value)

        self.create_subscription(String, self.cmd_topic, self.command_callback, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error("Could not connect to pigpio daemon. Run: sudo pigpiod")
            raise RuntimeError("pigpio daemon not connected")

        self.busy = False
        self.set_servo(SERVO_CENTER)
        self.get_logger().info(f"Bucket action node started. cmd={self.cmd_topic}, status={self.status_topic}")

    def set_servo(self, pulse):
        self.pi.set_servo_pulsewidth(SERVO_PIN, int(pulse))

    def slow_servo_move(self, start, end, duration=MOVE_DURATION, steps=STEPS):
        step_size = (end - start) / float(steps)
        delay = duration / float(steps)
        pulse = start
        for _ in range(steps + 1):
            self.set_servo(pulse)
            pulse += step_size
            time.sleep(delay)

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def do_dump_sequence(self):
        self.get_logger().info("Starting dump sequence")
        self.publish_status("busy")
        self.slow_servo_move(SERVO_CENTER, SERVO_DUMP, duration=MOVE_DURATION, steps=STEPS)
        time.sleep(HOLD_TIME)
        self.slow_servo_move(SERVO_DUMP, SERVO_CENTER, duration=MOVE_DURATION, steps=STEPS)
        self.get_logger().info("Dump sequence complete")

    def command_callback(self, msg):
        cmd = msg.data.strip().lower()

        if self.busy:
            self.get_logger().warn("Bucket node busy, ignoring command")
            return

        if cmd != "dump":
            self.get_logger().warn(f"Unknown bucket action: {cmd}")
            self.publish_status("failed")
            return

        self.busy = True
        try:
            self.do_dump_sequence()
            self.publish_status("done")
        except Exception as e:
            self.get_logger().error(f"Dump action failed: {e}")
            self.publish_status("failed")
        finally:
            self.busy = False

    def destroy_node(self):
        try:
            self.set_servo(0)
            self.pi.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BucketActionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
