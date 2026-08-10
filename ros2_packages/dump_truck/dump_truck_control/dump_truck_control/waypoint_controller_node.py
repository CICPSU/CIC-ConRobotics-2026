#!/usr/bin/env python3

import math
import os
import sys

import yaml

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from ament_index_python.packages import get_package_share_directory

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

    return math.atan2(
        siny_cosp,
        cosy_cosp,
    )


def resolve_waypoints_yaml(cli_argument=None):
    """
    Resolve the waypoint YAML file.

    Search order:

    1. Path supplied directly by the user.

    2. Source package waypoints directory.
       This allows YAML files to be added or edited without
       rebuilding the ROS package.

    3. Installed package share/waypoints directory.

    Example:

        ros2 run dump_truck_control \
          waypoint_controller_node \
          truck1 \
          demo_route.yaml
    """

    if not cli_argument:
        return None

    # ---------------------------------------------------------
    # 1. Explicit filesystem path
    # ---------------------------------------------------------

    explicit_path = os.path.abspath(
        os.path.expanduser(cli_argument)
    )

    if os.path.isfile(explicit_path):
        return explicit_path

    # ---------------------------------------------------------
    # 2. Source package waypoints directory
    #
    # IMPORTANT:
    # With --symlink-install, __file__ may point through the
    # installed symlink. realpath() follows that symlink back
    # to the actual source file.
    # ---------------------------------------------------------

    python_file = os.path.realpath(__file__)

    python_package_dir = os.path.dirname(
        python_file
    )

    package_root = os.path.dirname(
        python_package_dir
    )

    source_waypoints_dir = os.path.join(
        package_root,
        'waypoints',
    )

    source_waypoint_path = os.path.join(
        source_waypoints_dir,
        cli_argument,
    )

    if os.path.isfile(source_waypoint_path):
        return source_waypoint_path

    # ---------------------------------------------------------
    # 3. Installed ROS package share directory
    # ---------------------------------------------------------

    try:
        package_share = get_package_share_directory(
            'dump_truck_control'
        )

        installed_waypoint_path = os.path.join(
            package_share,
            'waypoints',
            cli_argument,
        )

        if os.path.isfile(installed_waypoint_path):
            return installed_waypoint_path

    except Exception:
        pass

    return None


class DumpTruckController(Node):

    def __init__(
        self,
        truck_name=None,
        cli_waypoints_yaml=None,
    ):
        super().__init__(
            'dump_truck_controller'
        )

        # -----------------------------------------------------
        # Truck identity
        # -----------------------------------------------------

        self.declare_parameter(
            'truck_name',
            '',
        )

        parameter_truck_name = str(
            self.get_parameter(
                'truck_name'
            ).value
        ).strip()

        if truck_name:
            self.truck_name = truck_name.strip().strip('/')

        elif parameter_truck_name:
            self.truck_name = parameter_truck_name.strip().strip('/')

        else:
            self.truck_name = ''

        # -----------------------------------------------------
        # Default topics
        # -----------------------------------------------------

        if self.truck_name:
            default_odom_topic = (
                f'/{self.truck_name}/fused_odom'
            )

            default_cmd_vel_topic = (
                f'/{self.truck_name}/cmd_vel'
            )

            default_bucket_cmd_topic = (
                f'/{self.truck_name}/bucket_action_cmd'
            )

            default_bucket_status_topic = (
                f'/{self.truck_name}/bucket_action_status'
            )

        else:
            default_odom_topic = '/fused_odom'
            default_cmd_vel_topic = '/cmd_vel'
            default_bucket_cmd_topic = '/bucket_action_cmd'
            default_bucket_status_topic = '/bucket_action_status'

        # -----------------------------------------------------
        # Topics
        # -----------------------------------------------------

        self.declare_parameter(
            'odom_topic',
            default_odom_topic,
        )

        self.declare_parameter(
            'cmd_vel_topic',
            default_cmd_vel_topic,
        )

        self.declare_parameter(
            'bucket_action_cmd_topic',
            default_bucket_cmd_topic,
        )

        self.declare_parameter(
            'bucket_action_status_topic',
            default_bucket_status_topic,
        )

        # -----------------------------------------------------
        # Waypoint YAML
        # -----------------------------------------------------

        self.declare_parameter(
            'waypoints_yaml',
            '',
        )

        # -----------------------------------------------------
        # Controller parameters
        # -----------------------------------------------------

        self.declare_parameter(
            'base_speed',
            0.35,
        )

        self.declare_parameter(
            'slow_speed',
            0.20,
        )

        self.declare_parameter(
            'turn_gain',
            0.9,
        )

        self.declare_parameter(
            'max_turn',
            0.35,
        )

        self.declare_parameter(
            'goal_tolerance',
            0.20,
        )

        # -----------------------------------------------------
        # Read parameters
        # -----------------------------------------------------

        self.odom_topic = self.get_parameter(
            'odom_topic'
        ).value

        self.cmd_vel_topic = self.get_parameter(
            'cmd_vel_topic'
        ).value

        self.bucket_action_cmd_topic = self.get_parameter(
            'bucket_action_cmd_topic'
        ).value

        self.bucket_action_status_topic = self.get_parameter(
            'bucket_action_status_topic'
        ).value

        parameter_yaml_path = self.get_parameter(
            'waypoints_yaml'
        ).value

        self.base_speed = float(
            self.get_parameter(
                'base_speed'
            ).value
        )

        self.slow_speed = float(
            self.get_parameter(
                'slow_speed'
            ).value
        )

        self.turn_gain = float(
            self.get_parameter(
                'turn_gain'
            ).value
        )

        self.max_turn = float(
            self.get_parameter(
                'max_turn'
            ).value
        )

        self.goal_tolerance = float(
            self.get_parameter(
                'goal_tolerance'
            ).value
        )

        # -----------------------------------------------------
        # Resolve waypoint YAML
        # -----------------------------------------------------

        yaml_path = None

        if cli_waypoints_yaml:
            yaml_path = resolve_waypoints_yaml(
                cli_waypoints_yaml
            )

            if yaml_path is None:
                self.get_logger().error(
                    'Waypoint YAML specified on the command line '
                    f'could not be found: {cli_waypoints_yaml}'
                )

        elif parameter_yaml_path:
            yaml_path = resolve_waypoints_yaml(
                parameter_yaml_path
            )

            if yaml_path is None:
                self.get_logger().error(
                    'Waypoint YAML specified by ROS parameter '
                    f'could not be found: {parameter_yaml_path}'
                )

        else:
            self.get_logger().error(
                'No waypoint YAML was specified.'
            )

        # -----------------------------------------------------
        # ROS communication
        # -----------------------------------------------------

        self.cmd_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10,
        )

        self.action_pub = self.create_publisher(
            String,
            self.bucket_action_cmd_topic,
            10,
        )

        self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
        )

        self.create_subscription(
            String,
            self.bucket_action_status_topic,
            self.action_status_callback,
            10,
        )

        # -----------------------------------------------------
        # Robot state
        # -----------------------------------------------------

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.pose_ready = False

        # -----------------------------------------------------
        # Load waypoints
        # -----------------------------------------------------

        self.waypoints = []

        if yaml_path is not None:
            try:
                with open(
                    yaml_path,
                    'r',
                    encoding='utf-8',
                ) as f:

                    data = yaml.safe_load(f) or {}

                raw_waypoints = data.get(
                    'waypoints',
                    [],
                )

                for pt in raw_waypoints:

                    if (
                        not isinstance(pt, list)
                        or len(pt) < 3
                    ):
                        self.get_logger().warn(
                            f'Skipping invalid waypoint: {pt}'
                        )
                        continue

                    x = float(pt[0])
                    y = float(pt[1])
                    direction = int(pt[2])

                    if direction not in [1, -1]:
                        self.get_logger().warn(
                            'Skipping invalid direction '
                            f'waypoint: {pt}'
                        )
                        continue

                    action = None

                    if (
                        len(pt) >= 4
                        and pt[3] is not None
                    ):
                        action = str(
                            pt[3]
                        ).strip().lower()

                    self.waypoints.append({
                        'x': x,
                        'y': y,
                        'direction': direction,
                        'action': action,
                    })

                self.get_logger().info(
                    f'Loaded {len(self.waypoints)} '
                    f'waypoints from YAML: {yaml_path}'
                )

            except Exception as exc:
                self.get_logger().error(
                    f'Failed to load YAML: {exc}'
                )

                self.waypoints = []

        # -----------------------------------------------------
        # Sequence state
        # -----------------------------------------------------

        self.current_goal_index = 0
        self.finished = False
        self.waiting_for_action = False
        self.current_action = None
        self.action_sent = False

        # -----------------------------------------------------
        # Control loop
        # -----------------------------------------------------

        self.timer = self.create_timer(
            0.1,
            self.control_loop,
        )

        self.get_logger().info(
            'Dump truck controller started. '
            f'truck={self.truck_name or "none"}, '
            f'odom={self.odom_topic}, '
            f'cmd_vel={self.cmd_vel_topic}, '
            f'bucket_cmd={self.bucket_action_cmd_topic}, '
            f'bucket_status={self.bucket_action_status_topic}'
        )

    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        self.yaw = quaternion_to_yaw(
            q.x,
            q.y,
            q.z,
            q.w,
        )

        self.pose_ready = True

    def action_status_callback(self, msg):

        status = msg.data.strip().lower()

        if status == 'busy':

            self.get_logger().info(
                'Bucket action node is busy',
                throttle_duration_sec=1.0,
            )

            return

        if (
            self.waiting_for_action
            and status == 'done'
        ):

            self.get_logger().info(
                f'Action complete: {self.current_action}'
            )

            self.waiting_for_action = False
            self.current_action = None
            self.action_sent = False

            self.current_goal_index += 1

            if (
                self.current_goal_index
                >= len(self.waypoints)
            ):

                self.finished = True
                self.stop_robot()

                self.get_logger().info(
                    'All waypoints and actions completed'
                )

        elif (
            self.waiting_for_action
            and status == 'failed'
        ):

            self.get_logger().error(
                'Bucket action failed. '
                'Robot will remain stopped.'
            )

            self.stop_robot()

    def stop_robot(self):

        self.cmd_pub.publish(
            Twist()
        )

    def publish_action(self, action_name):

        msg = String()
        msg.data = action_name

        self.action_pub.publish(
            msg
        )

    def control_loop(self):

        if not self.pose_ready:

            self.get_logger().info(
                'Waiting for fused odometry...',
                throttle_duration_sec=1.0,
            )

            return

        if (
            self.finished
            or self.current_goal_index
            >= len(self.waypoints)
        ):

            self.stop_robot()
            return

        if self.waiting_for_action:

            self.stop_robot()

            if not self.action_sent:

                self.publish_action(
                    self.current_action
                )

                self.action_sent = True

                self.get_logger().info(
                    'Sent action command: '
                    f'{self.current_action}'
                )

            return

        goal = self.waypoints[
            self.current_goal_index
        ]

        goal_x = goal['x']
        goal_y = goal['y']
        direction = goal['direction']
        action = goal['action']

        dx = goal_x - self.x
        dy = goal_y - self.y

        distance = math.sqrt(
            dx * dx + dy * dy
        )

        if distance < self.goal_tolerance:

            self.stop_robot()

            if action is not None:

                self.waiting_for_action = True
                self.current_action = action
                self.action_sent = False

                self.get_logger().info(
                    'Reached waypoint '
                    f'({goal_x:.2f}, {goal_y:.2f}), '
                    'performing action: '
                    f'{action}'
                )

                return

            else:

                self.current_goal_index += 1

                if (
                    self.current_goal_index
                    >= len(self.waypoints)
                ):

                    self.finished = True
                    self.stop_robot()

                    self.get_logger().info(
                        'All waypoints reached'
                    )

                    return

                goal = self.waypoints[
                    self.current_goal_index
                ]

                goal_x = goal['x']
                goal_y = goal['y']
                direction = goal['direction']
                action = goal['action']

                dx = goal_x - self.x
                dy = goal_y - self.y

                distance = math.sqrt(
                    dx * dx + dy * dy
                )

        target_angle = math.atan2(
            dy,
            dx,
        )

        if direction == -1:
            effective_yaw = normalize_angle(
                self.yaw + math.pi
            )
        else:
            effective_yaw = self.yaw

        heading_error = normalize_angle(
            target_angle - effective_yaw
        )

        cmd = Twist()

        if abs(heading_error) > 0.8:
            speed = self.slow_speed
        else:
            speed = self.base_speed

        if abs(heading_error) > 1.2:
            speed = 0.12

        cmd.linear.x = (
            direction * speed
        )

        cmd.angular.z = max(
            -self.max_turn,
            min(
                self.max_turn,
                self.turn_gain
                * heading_error,
            ),
        )

        self.cmd_pub.publish(
            cmd
        )

        self.get_logger().info(
            f'goal=({goal_x:.2f},{goal_y:.2f}) '
            f'pose=({self.x:.2f},{self.y:.2f},{self.yaw:.2f}) '
            f'dist={distance:.2f} '
            f'err={heading_error:.2f} '
            f'dir={direction} '
            f'action={action} '
            f'cmd_lin={cmd.linear.x:.2f} '
            f'cmd_ang={cmd.angular.z:.2f}',
            throttle_duration_sec=0.5,
        )


def main(args=None):

    raw_args = (
        sys.argv
        if args is None
        else args
    )

    non_ros_args = remove_ros_args(
        args=raw_args
    )

    truck_name = None
    cli_waypoints_yaml = None

    if len(non_ros_args) >= 2:

        candidate_truck = non_ros_args[1]

        if candidate_truck.startswith('truck'):
            truck_name = candidate_truck

    if len(non_ros_args) >= 3:

        candidate_yaml = non_ros_args[2]

        if candidate_yaml.endswith(
            (
                '.yaml',
                '.yml',
            )
        ):
            cli_waypoints_yaml = candidate_yaml

    rclpy.init(
        args=raw_args
    )

    node = DumpTruckController(
        truck_name=truck_name,
        cli_waypoints_yaml=cli_waypoints_yaml,
    )

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        if rclpy.ok():

            node.stop_robot()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':

    main()