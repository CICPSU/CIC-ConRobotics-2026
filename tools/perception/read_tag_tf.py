#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class TagTfReader(Node):
    def __init__(self):
        super().__init__('tag_tf_reader')

        self.parent_frame = 'camera'
        self.child_frame = 'tag36h11_0' # Change this based on the family of the April Tag

        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10
        )

        self.last_print_time = self.get_clock().now()
        self.print_interval_sec = 0.5

        self.get_logger().info('Started reading /tf ...')

    def tf_callback(self, msg: TFMessage):
        for t in msg.transforms:
            if t.header.frame_id != self.parent_frame:
                continue
            if t.child_frame_id != self.child_frame:
                continue

            x = t.transform.translation.x
            y = t.transform.translation.y
            z = t.transform.translation.z

            qx = t.transform.rotation.x
            qy = t.transform.rotation.y
            qz = t.transform.rotation.z
            qw = t.transform.rotation.w

            distance = math.sqrt(x*x + y*y + z*z)

            now = self.get_clock().now()
            dt = (now - self.last_print_time).nanoseconds / 1e9

            if dt >= self.print_interval_sec:
                print('------------------------------')
                print(f'parent frame : {self.parent_frame}')
                print(f'child frame  : {self.child_frame}')
                print(f'x = {x:.4f} m')
                print(f'y = {y:.4f} m')
                print(f'z = {z:.4f} m')
                print(f'distance = {distance:.4f} m')
                print(f'qx = {qx:.4f}')
                print(f'qy = {qy:.4f}')
                print(f'qz = {qz:.4f}')
                print(f'qw = {qw:.4f}')
                self.last_print_time = now


def main(args=None):
    rclpy.init(args=args)
    node = TagTfReader()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
