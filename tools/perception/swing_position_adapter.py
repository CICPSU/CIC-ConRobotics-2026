#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage


class SwingPositionAdapter(Node):
    """Convert one sensor-specific TF into canonical swing JointState."""

    def __init__(self):
        super().__init__("swing_position_adapter")
        self.declare_parameter("tag_frame", "tag36h11_9")
        self.declare_parameter(
            "output_topic",
            "/excavator3/swing_joint_state",
        )
        self.declare_parameter("print_rate_hz", 5.0)
        # Fixed transform from the ceiling camera yaw convention to the
        # site convention used by operations: left=0 deg and up=+90 deg.
        # This is not automatic zeroing and does not depend on startup pose.
        self.declare_parameter("camera_to_site_yaw_offset_deg", -90.0)

        self.tag_frame = str(self.get_parameter("tag_frame").value).lstrip("/")
        self.output_topic = str(self.get_parameter("output_topic").value)
        print_rate_hz = max(0.1, float(self.get_parameter("print_rate_hz").value))
        self.camera_to_site_yaw_offset_rad = math.radians(
            float(
                self.get_parameter(
                    "camera_to_site_yaw_offset_deg"
                ).value
            )
        )

        self._last_raw_yaw_rad = None
        self._last_yaw_rad = None
        self._last_receive = None
        self._last_source_stamp = None

        tf_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.publisher = self.create_publisher(
            JointState,
            self.output_topic,
            output_qos,
        )
        self.subscription = self.create_subscription(
            TFMessage,
            "/tf",
            self._tf_callback,
            tf_qos,
        )
        self.status_timer = self.create_timer(
            1.0 / print_rate_hz,
            self._print_status,
        )

        self.get_logger().info(
            f"Converting {self.tag_frame} TF to {self.output_topic}"
        )
        self.get_logger().info(
            "Publishing site-frame swing yaw with fixed camera-to-site "
            f"offset {math.degrees(self.camera_to_site_yaw_offset_rad):+.2f} deg; "
            "no automatic zeroing."
        )

    @staticmethod
    def _quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
        sin_yaw = 2.0 * ((w * z) + (x * y))
        cos_yaw = 1.0 - 2.0 * ((y * y) + (z * z))
        return math.atan2(sin_yaw, cos_yaw)

    def _tf_callback(self, message: TFMessage) -> None:
        for transform in message.transforms:
            if transform.child_frame_id.lstrip("/") != self.tag_frame:
                continue

            rotation = transform.transform.rotation
            raw_yaw_rad = self._quaternion_to_yaw(
                rotation.x,
                rotation.y,
                rotation.z,
                rotation.w,
            )
            yaw_rad = math.atan2(
                math.sin(
                    raw_yaw_rad + self.camera_to_site_yaw_offset_rad
                ),
                math.cos(
                    raw_yaw_rad + self.camera_to_site_yaw_offset_rad
                ),
            )

            output = JointState()
            output.header = transform.header
            output.name = ["swing_joint"]
            output.position = [yaw_rad]
            self.publisher.publish(output)

            self._last_raw_yaw_rad = raw_yaw_rad
            self._last_yaw_rad = yaw_rad
            self._last_receive = time.monotonic()
            self._last_source_stamp = transform.header.stamp

    def _print_status(self) -> None:
        if self._last_receive is None:
            self.get_logger().warning("Waiting for swing Tag TF...")
            return

        receive_age = time.monotonic() - self._last_receive
        now_ns = self.get_clock().now().nanoseconds
        source_ns = (
            int(self._last_source_stamp.sec) * 1_000_000_000
            + int(self._last_source_stamp.nanosec)
        )
        source_age_ms = max(0.0, (now_ns - source_ns) / 1_000_000.0)
        self.get_logger().info(
            f"camera_yaw={math.degrees(self._last_raw_yaw_rad):7.2f} deg | "
            f"site_swing={math.degrees(self._last_yaw_rad):7.2f} deg | "
            f"source_age={source_age_ms:6.1f} ms | "
            f"receive_age={receive_age * 1000.0:5.1f} ms"
        )


def main(args=None):
    rclpy.init(args=args)
    node = SwingPositionAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()