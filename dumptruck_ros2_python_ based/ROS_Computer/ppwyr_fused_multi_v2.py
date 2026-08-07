#!/usr/bin/env python3
import math
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def quaternion_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class DumpTruckController(Node):
    def __init__(self):
        super().__init__('dump_truck_controller')

        self.declare_parameter('odom_topic', '/fused_odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('bucket_action_cmd_topic', '/bucket_action_cmd')
        self.declare_parameter('bucket_action_status_topic', '/bucket_action_status')
        self.declare_parameter('waypoints_yaml', '/home/mre5349/Diff_Drive/waypoints.yaml')

        self.declare_parameter('base_speed', 0.35)
        self.declare_parameter('slow_speed', 0.20)
        self.declare_parameter('turn_gain', 0.9)
        self.declare_parameter('max_turn', 0.35)
        self.declare_parameter('goal_tolerance', 0.20)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.bucket_action_cmd_topic = self.get_parameter('bucket_action_cmd_topic').value
        self.bucket_action_status_topic = self.get_parameter('bucket_action_status_topic').value
        yaml_path = self.get_parameter('waypoints_yaml').value

        self.base_speed = float(self.get_parameter('base_speed').value)
        self.slow_speed = float(self.get_parameter('slow_speed').value)
        self.turn_gain = float(self.get_parameter('turn_gain').value)
        self.max_turn = float(self.get_parameter('max_turn').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.action_pub = self.create_publisher(String, self.bucket_action_cmd_topic, 10)

        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.create_subscription(String, self.bucket_action_status_topic, self.action_status_callback, 10)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.pose_ready = False

        self.waypoints = []
        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
                raw_waypoints = data.get('waypoints', [])

                for pt in raw_waypoints:
                    if not isinstance(pt, list) or len(pt) < 3:
                        self.get_logger().warn(f'Skipping invalid waypoint: {pt}')
                        continue

                    x = float(pt[0])
                    y = float(pt[1])
                    direction = int(pt[2])
                    if direction not in [1, -1]:
                        self.get_logger().warn(f'Skipping invalid direction waypoint: {pt}')
                        continue

                    action = None
                    if len(pt) >= 4 and pt[3] is not None:
                        action = str(pt[3]).strip().lower()

                    self.waypoints.append({
                        'x': x,
                        'y': y,
                        'direction': direction,
                        'action': action,
                    })

                self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints from YAML: {yaml_path}')

        except Exception as e:
            self.get_logger().error(f'Failed to load YAML: {e}')
            self.waypoints = []

        self.current_goal_index = 0
        self.finished = False
        self.waiting_for_action = False
        self.current_action = None
        self.action_sent = False

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f'Dump truck controller started. odom={self.odom_topic}, cmd_vel={self.cmd_vel_topic}, bucket_cmd={self.bucket_action_cmd_topic}, bucket_status={self.bucket_action_status_topic}')

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.pose_ready = True

    def action_status_callback(self, msg):
        status = msg.data.strip().lower()

        if status == 'busy':
            self.get_logger().info('Bucket action node is busy', throttle_duration_sec=1.0)
            return

        if self.waiting_for_action and status == 'done':
            self.get_logger().info(f'Action complete: {self.current_action}')
            self.waiting_for_action = False
            self.current_action = None
            self.action_sent = False
            self.current_goal_index += 1

            if self.current_goal_index >= len(self.waypoints):
                self.finished = True
                self.stop_robot()
                self.get_logger().info('All waypoints and actions completed')

        elif self.waiting_for_action and status == 'failed':
            self.get_logger().error('Bucket action failed. Robot will remain stopped.')
            self.stop_robot()

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def publish_action(self, action_name):
        msg = String()
        msg.data = action_name
        self.action_pub.publish(msg)

    def control_loop(self):
        if not self.pose_ready:
            self.get_logger().info('Waiting for fused odometry...', throttle_duration_sec=1.0)
            return

        if self.finished or self.current_goal_index >= len(self.waypoints):
            self.stop_robot()
            return

        if self.waiting_for_action:
            self.stop_robot()
            if not self.action_sent:
                self.publish_action(self.current_action)
                self.action_sent = True
                self.get_logger().info(f'Sent action command: {self.current_action}')
            return

        goal = self.waypoints[self.current_goal_index]
        goal_x = goal['x']
        goal_y = goal['y']
        direction = goal['direction']
        action = goal['action']

        dx = goal_x - self.x
        dy = goal_y - self.y
        distance = math.sqrt(dx * dx + dy * dy)

        if distance < self.goal_tolerance:
            self.stop_robot()

            if action is not None:
                self.waiting_for_action = True
                self.current_action = action
                self.action_sent = False
                self.get_logger().info(
                    f'Reached waypoint ({goal_x:.2f}, {goal_y:.2f}), performing action: {action}'
                )
                return
            else:
                self.current_goal_index += 1

                if self.current_goal_index >= len(self.waypoints):
                    self.finished = True
                    self.stop_robot()
                    self.get_logger().info('All waypoints reached')
                    return

                goal = self.waypoints[self.current_goal_index]
                goal_x = goal['x']
                goal_y = goal['y']
                direction = goal['direction']
                action = goal['action']
                dx = goal_x - self.x
                dy = goal_y - self.y
                distance = math.sqrt(dx * dx + dy * dy)

        target_angle = math.atan2(dy, dx)
        effective_yaw = normalize_angle(self.yaw + math.pi) if direction == -1 else self.yaw
        heading_error = normalize_angle(target_angle - effective_yaw)

        cmd = Twist()
        speed = self.slow_speed if abs(heading_error) > 0.8 else self.base_speed
        if abs(heading_error) > 1.2:
            speed = 0.12

        cmd.linear.x = direction * speed
        cmd.angular.z = max(-self.max_turn, min(self.max_turn, self.turn_gain * heading_error))
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'goal=({goal_x:.2f},{goal_y:.2f}) pose=({self.x:.2f},{self.y:.2f},{self.yaw:.2f}) '
            f'dist={distance:.2f} err={heading_error:.2f} dir={direction} action={action} '
            f'cmd_lin={cmd.linear.x:.2f} cmd_ang={cmd.angular.z:.2f}',
            throttle_duration_sec=0.5
        )


def main(args=None):
    rclpy.init(args=args)
    node = DumpTruckController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop_robot()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
