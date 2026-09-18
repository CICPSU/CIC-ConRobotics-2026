#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from std_msgs.msg import Bool, Float64
from tf2_msgs.msg import TFMessage


class SwingTagMonitor(Node):
    """
    Monitor Excavator 3 swing angle using AprilTag ID 9.

    The TF yaw is used directly without zeroing or offset correction.

    Published topics:
        /excavator3/swing_angle_deg
        /excavator3/swing_tag_visible
    """

    def __init__(self):
        super().__init__('swing_tag_monitor')

        # ---------------------------------------------------------
        # Parameters
        # ---------------------------------------------------------

        self.declare_parameter('tag_frame', 'tag36h11_9')

        self.declare_parameter(
            'angle_topic',
            '/excavator3/swing_angle_deg',
        )

        self.declare_parameter(
            'visible_topic',
            '/excavator3/swing_tag_visible',
        )

        self.declare_parameter('lost_timeout_sec', 0.30)
        self.declare_parameter('print_rate_hz', 5.0)

        self.tag_frame = (
            self.get_parameter('tag_frame')
            .get_parameter_value()
            .string_value
        )

        self.angle_topic = (
            self.get_parameter('angle_topic')
            .get_parameter_value()
            .string_value
        )

        self.visible_topic = (
            self.get_parameter('visible_topic')
            .get_parameter_value()
            .string_value
        )

        self.lost_timeout_sec = (
            self.get_parameter('lost_timeout_sec')
            .get_parameter_value()
            .double_value
        )

        self.print_rate_hz = (
            self.get_parameter('print_rate_hz')
            .get_parameter_value()
            .double_value
        )

        # ---------------------------------------------------------
        # Runtime state
        # ---------------------------------------------------------

        self.current_yaw_deg: Optional[float] = None
        self.last_transform_stamp_sec: Optional[float] = None
        self.last_receive_time_sec: Optional[float] = None

        self.tag_visible = False
        self.previous_visible_state: Optional[bool] = None

        # ---------------------------------------------------------
        # QoS for /tf
        # ---------------------------------------------------------

        tf_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ---------------------------------------------------------
        # ROS interfaces
        # ---------------------------------------------------------

        self.tf_subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            tf_qos,
        )

        self.angle_publisher = self.create_publisher(
            Float64,
            self.angle_topic,
            10,
        )

        self.visible_publisher = self.create_publisher(
            Bool,
            self.visible_topic,
            10,
        )

        timer_period = 1.0 / max(self.print_rate_hz, 0.1)

        self.status_timer = self.create_timer(
            timer_period,
            self.status_timer_callback,
        )

        self.get_logger().info(
            f'Monitoring TF frame: {self.tag_frame}'
        )

        self.get_logger().info(
            'Publishing the raw TF yaw without zeroing.'
        )

    # -------------------------------------------------------------
    # Angle conversion
    # -------------------------------------------------------------

    @staticmethod
    def quaternion_to_yaw_deg(x, y, z, w) -> float:
        """
        Convert a quaternion to Euler yaw in degrees.

        Output range:
            -180 <= yaw <= 180
        """

        sin_yaw = 2.0 * ((w * z) + (x * y))
        cos_yaw = 1.0 - 2.0 * ((y * y) + (z * z))

        yaw_rad = math.atan2(sin_yaw, cos_yaw)

        return math.degrees(yaw_rad)

    def ros_time_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    # -------------------------------------------------------------
    # TF processing
    # -------------------------------------------------------------

    def tf_callback(self, message: TFMessage):
        for transform in message.transforms:

            child_frame = transform.child_frame_id.lstrip('/')

            if child_frame != self.tag_frame.lstrip('/'):
                continue

            rotation = transform.transform.rotation

            yaw_deg = self.quaternion_to_yaw_deg(
                rotation.x,
                rotation.y,
                rotation.z,
                rotation.w,
            )

            stamp = transform.header.stamp

            transform_stamp_sec = (
                float(stamp.sec)
                + float(stamp.nanosec) / 1_000_000_000.0
            )

            self.current_yaw_deg = yaw_deg
            self.last_transform_stamp_sec = transform_stamp_sec
            self.last_receive_time_sec = self.ros_time_sec()

            angle_message = Float64()
            angle_message.data = self.current_yaw_deg
            self.angle_publisher.publish(angle_message)

    # -------------------------------------------------------------
    # Status and Tag-loss monitoring
    # -------------------------------------------------------------

    def status_timer_callback(self):
        now_sec = self.ros_time_sec()

        if self.last_receive_time_sec is None:
            self.tag_visible = False
            receive_age_sec = float('inf')
        else:
            receive_age_sec = (
                now_sec - self.last_receive_time_sec
            )

            self.tag_visible = (
                receive_age_sec <= self.lost_timeout_sec
            )

        visible_message = Bool()
        visible_message.data = self.tag_visible
        self.visible_publisher.publish(visible_message)

        if self.tag_visible != self.previous_visible_state:
            if self.tag_visible:
                self.get_logger().info('Tag detected.')
            else:
                self.get_logger().warning('Tag lost.')

            self.previous_visible_state = self.tag_visible

        if not self.tag_visible:
            return

        if (
            self.current_yaw_deg is None
            or self.last_transform_stamp_sec is None
        ):
            return

        message_age_sec = max(
            0.0,
            now_sec - self.last_transform_stamp_sec,
        )

        self.get_logger().info(
            f'swing_yaw={self.current_yaw_deg:7.2f} deg | '
            f'message_age={message_age_sec * 1000.0:6.1f} ms'
        )


def main(args=None):
    rclpy.init(args=args)

    node = SwingTagMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()