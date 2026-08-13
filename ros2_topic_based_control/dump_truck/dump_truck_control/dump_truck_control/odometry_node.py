#!/usr/bin/env python3
"""
odometry_node.py

Wheel encoder odometry for one dump truck.

Input:
  /truckN/wheel_states

Output:
  /truckN/odom

This version calculates odometry from the CHANGE IN ENCODER POSITION
(tick counts), rather than directly integrating msg.velocity.

Why:
The wheel state publisher runs much faster than encoder ticks arrive.
Therefore, during straight motion, messages often look like:

    velocity = [50, 0]
    velocity = [0, 50]

even though both wheels are physically moving at nearly the same speed.

Using those instantaneous velocity values directly produces artificial
angular velocity spikes.

Instead, this node uses:

    delta_left_ticks
    delta_right_ticks

from JointState.position.

TF publishing is optional. For multi-truck operation, normally leave
publish_tf:=false unless unique TF frame names are being used.
"""

import math

import rclpy

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState

import tf2_ros


class OdometryNode(Node):

    def __init__(self):

        super().__init__('dump_truck_odometry')

        # =====================================================
        # ROS Parameters
        # =====================================================

        self.declare_parameter(
            'wheel_states_topic',
            '/wheel_states',
        )

        self.declare_parameter(
            'odom_topic',
            '/odom',
        )

        self.declare_parameter(
            'odom_frame',
            'odom',
        )

        self.declare_parameter(
            'base_frame',
            'base_link',
        )

        self.declare_parameter(
            'publish_tf',
            False,
        )

        # =====================================================
        # Robot Geometry / Encoder Calibration
        # =====================================================

        self.declare_parameter(
            'wheel_radius',
            0.045,
        )

        self.declare_parameter(
            'wheelbase',
            0.178,
        )

        self.declare_parameter(
            'ticks_per_rev',
            70.0,
        )

        # =====================================================
        # Read Parameters
        # =====================================================

        self.wheel_states_topic = str(
            self.get_parameter(
                'wheel_states_topic'
            ).value
        )

        self.odom_topic = str(
            self.get_parameter(
                'odom_topic'
            ).value
        )

        self.odom_frame = str(
            self.get_parameter(
                'odom_frame'
            ).value
        )

        self.base_frame = str(
            self.get_parameter(
                'base_frame'
            ).value
        )

        self.publish_tf = bool(
            self.get_parameter(
                'publish_tf'
            ).value
        )

        self.wheel_radius = float(
            self.get_parameter(
                'wheel_radius'
            ).value
        )

        self.wheelbase = float(
            self.get_parameter(
                'wheelbase'
            ).value
        )

        self.ticks_per_rev = float(
            self.get_parameter(
                'ticks_per_rev'
            ).value
        )

        # =====================================================
        # Encoder Conversion
        # =====================================================

        self.meters_per_tick = (
            2.0
            * math.pi
            * self.wheel_radius
            / self.ticks_per_rev
        )

        # =====================================================
        # Robot Pose
        # =====================================================

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # =====================================================
        # Previous Encoder State
        # =====================================================

        self.prev_left_ticks = None
        self.prev_right_ticks = None

        self.prev_time = None

        # =====================================================
        # Latest Velocity Estimate
        # =====================================================

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # =====================================================
        # ROS Interfaces
        # =====================================================

        self.create_subscription(
            JointState,
            self.wheel_states_topic,
            self.wheel_callback,
            10,
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            self.odom_topic,
            10,
        )

        self.tf_broadcaster = (
            tf2_ros.TransformBroadcaster(
                self
            )
        )

        # =====================================================
        # Startup Log
        # =====================================================

        self.get_logger().info(
            'Odometry node started. '
            f'wheel_states={self.wheel_states_topic}, '
            f'odom={self.odom_topic}, '
            f'wheel_radius={self.wheel_radius:.4f}, '
            f'wheelbase={self.wheelbase:.4f}, '
            f'ticks_per_rev={self.ticks_per_rev:.1f}, '
            f'meters_per_tick={self.meters_per_tick:.6f}, '
            f'publish_tf={self.publish_tf}, '
            'integration=encoder_position_delta'
        )

    # =========================================================
    # Wheel State Callback
    # =========================================================

    def wheel_callback(
        self,
        msg,
    ):

        # -----------------------------------------------------
        # Make sure two wheel positions exist
        # -----------------------------------------------------

        if len(msg.position) < 2:

            self.get_logger().warn(
                'wheel_states position field '
                'does not contain two entries'
            )

            return

        # -----------------------------------------------------
        # Read current absolute encoder tick counts
        # -----------------------------------------------------

        current_left_ticks = float(
            msg.position[0]
        )

        current_right_ticks = float(
            msg.position[1]
        )

        now = self.get_clock().now()

        # -----------------------------------------------------
        # First message:
        #
        # Establish encoder baseline only.
        #
        # This prevents the initial encoder values
        # (e.g. 693, 689) from being interpreted as robot
        # movement.
        # -----------------------------------------------------

        if (
            self.prev_left_ticks is None
            or self.prev_right_ticks is None
            or self.prev_time is None
        ):

            self.prev_left_ticks = (
                current_left_ticks
            )

            self.prev_right_ticks = (
                current_right_ticks
            )

            self.prev_time = now

            self.publish_odometry(
                now,
                0.0,
                0.0,
            )

            self.get_logger().info(
                'Encoder baseline initialized: '
                f'left={current_left_ticks:.0f}, '
                f'right={current_right_ticks:.0f}'
            )

            return

        # -----------------------------------------------------
        # Time Difference
        # -----------------------------------------------------

        dt = (
            now
            - self.prev_time
        ).nanoseconds / 1e9

        if dt <= 0.0:
            return

        # -----------------------------------------------------
        # Encoder Tick Differences
        # -----------------------------------------------------

        delta_left_ticks = (
            current_left_ticks
            - self.prev_left_ticks
        )

        delta_right_ticks = (
            current_right_ticks
            - self.prev_right_ticks
        )

        # -----------------------------------------------------
        # Convert Encoder Ticks to Wheel Travel
        # -----------------------------------------------------

        delta_left = (
            delta_left_ticks
            * self.meters_per_tick
        )

        delta_right = (
            delta_right_ticks
            * self.meters_per_tick
        )

        # -----------------------------------------------------
        # Differential Drive Motion
        #
        # ds:
        #   forward displacement of robot center
        #
        # dtheta:
        #   change in robot yaw
        # -----------------------------------------------------

        delta_distance = (
            delta_right
            + delta_left
        ) / 2.0

        delta_theta = (
            delta_right
            - delta_left
        ) / self.wheelbase

        # -----------------------------------------------------
        # Integrate Pose
        #
        # Use midpoint heading for slightly better differential
        # drive integration during turns.
        # -----------------------------------------------------

        theta_mid = (
            self.theta
            + delta_theta / 2.0
        )

        self.x += (
            delta_distance
            * math.cos(theta_mid)
        )

        self.y += (
            delta_distance
            * math.sin(theta_mid)
        )

        self.theta += delta_theta

        # Normalize yaw to [-pi, pi]
        self.theta = (
            self.theta
            + math.pi
        ) % (
            2.0
            * math.pi
        ) - math.pi

        # -----------------------------------------------------
        # Velocity Estimates
        #
        # These are now derived from encoder displacement,
        # rather than JointState.velocity.
        # -----------------------------------------------------

        self.linear_velocity = (
            delta_distance
            / dt
        )

        self.angular_velocity = (
            delta_theta
            / dt
        )

        # -----------------------------------------------------
        # Publish Odometry
        # -----------------------------------------------------

        self.publish_odometry(
            now,
            self.linear_velocity,
            self.angular_velocity,
        )

        # -----------------------------------------------------
        # Save State
        # -----------------------------------------------------

        self.prev_left_ticks = (
            current_left_ticks
        )

        self.prev_right_ticks = (
            current_right_ticks
        )

        self.prev_time = now

        # -----------------------------------------------------
        # Diagnostic Log
        # -----------------------------------------------------

        self.get_logger().info(
            f'{self.odom_topic}: '
            f'x={self.x:.3f}, '
            f'y={self.y:.3f}, '
            f'th={self.theta:.3f}, '
            f'dL_ticks={delta_left_ticks:.0f}, '
            f'dR_ticks={delta_right_ticks:.0f}, '
            f'v={self.linear_velocity:.3f}, '
            f'w={self.angular_velocity:.3f}',
            throttle_duration_sec=0.5,
        )

    # =========================================================
    # Odometry Publisher
    # =========================================================

    def publish_odometry(
        self,
        now,
        linear_velocity,
        angular_velocity,
    ):

        # -----------------------------------------------------
        # Odometry Message
        # -----------------------------------------------------

        odom = Odometry()

        odom.header.stamp = (
            now.to_msg()
        )

        odom.header.frame_id = (
            self.odom_frame
        )

        odom.child_frame_id = (
            self.base_frame
        )

        # -----------------------------------------------------
        # Position
        # -----------------------------------------------------

        odom.pose.pose.position.x = (
            self.x
        )

        odom.pose.pose.position.y = (
            self.y
        )

        odom.pose.pose.position.z = 0.0

        # -----------------------------------------------------
        # Orientation
        #
        # Yaw-only quaternion
        # -----------------------------------------------------

        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0

        odom.pose.pose.orientation.z = (
            math.sin(
                self.theta / 2.0
            )
        )

        odom.pose.pose.orientation.w = (
            math.cos(
                self.theta / 2.0
            )
        )

        # -----------------------------------------------------
        # Velocity
        # -----------------------------------------------------

        odom.twist.twist.linear.x = (
            linear_velocity
        )

        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0

        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0

        odom.twist.twist.angular.z = (
            angular_velocity
        )

        # -----------------------------------------------------
        # Publish
        # -----------------------------------------------------

        self.odom_pub.publish(
            odom
        )

        # -----------------------------------------------------
        # Optional TF
        # -----------------------------------------------------

        if self.publish_tf:

            transform = TransformStamped()

            transform.header.stamp = (
                odom.header.stamp
            )

            transform.header.frame_id = (
                self.odom_frame
            )

            transform.child_frame_id = (
                self.base_frame
            )

            transform.transform.translation.x = (
                self.x
            )

            transform.transform.translation.y = (
                self.y
            )

            transform.transform.translation.z = (
                0.0
            )

            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0

            transform.transform.rotation.z = (
                odom.pose.pose.orientation.z
            )

            transform.transform.rotation.w = (
                odom.pose.pose.orientation.w
            )

            self.tf_broadcaster.sendTransform(
                transform
            )


# =============================================================
# Main
# =============================================================

def main(args=None):

    rclpy.init(
        args=args
    )

    node = OdometryNode()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:
        pass

    finally:

        if rclpy.ok():

            node.destroy_node()

            rclpy.shutdown()


if __name__ == '__main__':
    main()