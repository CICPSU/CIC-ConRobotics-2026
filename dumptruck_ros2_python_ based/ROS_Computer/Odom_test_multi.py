#!/usr/bin/env python3
"""
Odom_test_multi.py

Wheel-state odometry for one dump truck.

Input:
  /truckN/wheel_states

Output:
  /truckN/odom

TF publishing is optional. For multi-truck operation, leave publish_tf:=false unless you
also make odom_frame/base_frame unique, e.g. truck1/odom and truck1/base_link.
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


class OdometryNode(Node):
    def __init__(self):
        super().__init__('dump_truck_odometry')

        self.declare_parameter('wheel_states_topic', '/wheel_states')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', False)

        self.declare_parameter('wheel_radius', 0.045)
        self.declare_parameter('wheelbase', 0.178)
        self.declare_parameter('ticks_per_rev', 70.0)
        self.declare_parameter('velocity_is_rads', False)

        self.wheel_states_topic = str(self.get_parameter('wheel_states_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.ticks_per_rev = float(self.get_parameter('ticks_per_rev').value)
        self.velocity_is_rads = bool(self.get_parameter('velocity_is_rads').value)
        self.meters_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.vel_L = 0.0
        self.vel_R = 0.0
        self.last_time = self.get_clock().now()

        self.create_subscription(JointState, self.wheel_states_topic, self.wheel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.create_timer(0.02, self.update_odometry)

        self.get_logger().info(
            f"Odometry node started. wheel_states={self.wheel_states_topic}, odom={self.odom_topic}, publish_tf={self.publish_tf}"
        )

    def wheel_callback(self, msg):
        if len(msg.velocity) < 2:
            self.get_logger().warn("wheel_states velocity field does not contain two entries")
            return
        self.vel_L = msg.velocity[0]
        self.vel_R = msg.velocity[1]

    def update_odometry(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return

        if self.velocity_is_rads:
            vL = self.vel_L * self.wheel_radius
            vR = self.vel_R * self.wheel_radius
        else:
            vL = self.vel_L * self.meters_per_tick
            vR = self.vel_R * self.meters_per_tick

        v = (vR + vL) / 2.0
        w = (vR - vL) / self.wheelbase

        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += w * dt
        self.theta = (self.theta + math.pi) % (2.0 * math.pi) - math.pi

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = odom.header.stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.z = odom.pose.pose.orientation.z
            t.transform.rotation.w = odom.pose.pose.orientation.w
            self.tf_broadcaster.sendTransform(t)

        self.get_logger().info(
            f"{self.odom_topic}: x={self.x:.3f}, y={self.y:.3f}, th={self.theta:.3f}, v={v:.3f}, w={w:.3f}",
            throttle_duration_sec=0.5
        )

        self.last_time = now


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
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
