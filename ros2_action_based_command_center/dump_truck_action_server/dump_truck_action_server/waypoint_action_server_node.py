#!/usr/bin/env python3

import math
import os
import time

import yaml

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from construction_site_interfaces.action import ExecuteRobotTask


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


def resolve_waypoints_yaml(task_file):
    """
    Resolve waypoint YAML.

    Search order:

    1. Explicit filesystem path.

    2. Source tree:
       ros2_topic_based_control/
       dump_truck/
       dump_truck_control/
       waypoints/

    3. Installed dump_truck_control package share directory.
    """

    if not task_file:
        return None

    # ---------------------------------------------------------
    # 1. Explicit filesystem path
    # ---------------------------------------------------------

    explicit_path = os.path.abspath(
        os.path.expanduser(task_file)
    )

    if os.path.isfile(explicit_path):
        return explicit_path

    # ---------------------------------------------------------
    # 2. Source tree
    #
    # Find repository root by walking upward from this file.
    # ---------------------------------------------------------

    current_path = os.path.realpath(__file__)
    search_dir = os.path.dirname(current_path)

    while True:
        candidate = os.path.join(
            search_dir,
            'ros2_topic_based_control',
            'dump_truck',
            'dump_truck_control',
            'waypoints',
            task_file,
        )

        if os.path.isfile(candidate):
            return candidate

        parent = os.path.dirname(search_dir)

        if parent == search_dir:
            break

        search_dir = parent

    # ---------------------------------------------------------
    # 3. Installed dump_truck_control package
    # ---------------------------------------------------------

    try:
        package_share = get_package_share_directory(
            'dump_truck_control'
        )

        installed_path = os.path.join(
            package_share,
            'waypoints',
            task_file,
        )

        if os.path.isfile(installed_path):
            return installed_path

    except Exception:
        pass

    return None


def load_waypoints(yaml_path):
    """
    Load and validate waypoint YAML.

    Expected waypoint format:

        waypoints:
          - [x, y, direction]
          - [x, y, direction, action]

    direction:
        1  = forward
        -1 = reverse
    """

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

    waypoints = []

    for pt in raw_waypoints:

        if (
            not isinstance(pt, list)
            or len(pt) < 3
        ):
            continue

        x = float(pt[0])
        y = float(pt[1])
        direction = int(pt[2])

        if direction not in [1, -1]:
            continue

        action = None

        if (
            len(pt) >= 4
            and pt[3] is not None
        ):
            action = str(
                pt[3]
            ).strip().lower()

        waypoints.append({
            'x': x,
            'y': y,
            'direction': direction,
            'action': action,
        })

    return waypoints


class DumpTruckWaypointActionServer(Node):

    def __init__(self):
        super().__init__(
            'waypoint_action_server'
        )

        self.callback_group = ReentrantCallbackGroup()

        # -----------------------------------------------------
        # Truck identity
        # -----------------------------------------------------

        self.declare_parameter(
            'truck_name',
            'truck1',
        )

        self.truck_name = str(
            self.get_parameter(
                'truck_name'
            ).value
        ).strip().strip('/')

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
        # Truck-specific topics
        # -----------------------------------------------------

        self.odom_topic = (
            f'/{self.truck_name}/fused_odom'
        )

        self.cmd_vel_topic = (
            f'/{self.truck_name}/cmd_vel'
        )

        self.bucket_action_cmd_topic = (
            f'/{self.truck_name}/bucket_action_cmd'
        )

        self.bucket_action_status_topic = (
            f'/{self.truck_name}/bucket_action_status'
        )

        self.action_name = (
            f'/{self.truck_name}/execute_robot_task'
        )

        # -----------------------------------------------------
        # ROS publishers / subscribers
        # -----------------------------------------------------

        self.cmd_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10,
        )

        self.bucket_action_pub = self.create_publisher(
            String,
            self.bucket_action_cmd_topic,
            10,
        )

        self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
            callback_group=self.callback_group,
        )

        self.create_subscription(
            String,
            self.bucket_action_status_topic,
            self.bucket_status_callback,
            10,
            callback_group=self.callback_group,
        )

        # -----------------------------------------------------
        # Robot state
        # -----------------------------------------------------

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.pose_ready = False

        # -----------------------------------------------------
        # Bucket state
        # -----------------------------------------------------

        self.bucket_status = ''
        self.waiting_for_bucket_action = False
        self.current_bucket_action = None

        # -----------------------------------------------------
        # Action execution state
        # -----------------------------------------------------

        self.task_active = False

        # -----------------------------------------------------
        # ROS 2 Action Server
        # -----------------------------------------------------

        self.action_server = ActionServer(
            self,
            ExecuteRobotTask,
            self.action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group,
        )

        self.get_logger().info(
            'Dump truck waypoint Action Server started. '
            f'truck={self.truck_name}, '
            f'action={self.action_name}, '
            f'odom={self.odom_topic}, '
            f'cmd_vel={self.cmd_vel_topic}'
        )

    # =========================================================
    # ROS callbacks
    # =========================================================

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

    def bucket_status_callback(self, msg):

        self.bucket_status = (
            msg.data.strip().lower()
        )

    # =========================================================
    # Action callbacks
    # =========================================================

    def goal_callback(self, goal_request):

        requested_robot = (
            goal_request.robot_name
            .strip()
            .strip('/')
        )

        task_type = (
            goal_request.task_type
            .strip()
            .lower()
        )

        # -----------------------------------------------------
        # Goal must belong to this truck
        # -----------------------------------------------------

        if requested_robot != self.truck_name:

            self.get_logger().warn(
                'Rejecting goal: '
                f'requested robot={requested_robot}, '
                f'this server={self.truck_name}'
            )

            return GoalResponse.REJECT

        # -----------------------------------------------------
        # This server currently supports waypoint tasks only
        # -----------------------------------------------------

        if task_type != 'waypoint':

            self.get_logger().warn(
                'Rejecting unsupported task type: '
                f'{task_type}'
            )

            return GoalResponse.REJECT

        # -----------------------------------------------------
        # Only one task at a time
        # -----------------------------------------------------

        if self.task_active:

            self.get_logger().warn(
                'Rejecting goal because another task '
                'is already active.'
            )

            return GoalResponse.REJECT

        self.get_logger().info(
            'Accepted task request: '
            f'robot={requested_robot}, '
            f'type={task_type}, '
            f'file={goal_request.task_file}'
        )

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):

        self.get_logger().warn(
            f'Cancel requested for {self.truck_name}'
        )

        return CancelResponse.ACCEPT

    # =========================================================
    # Robot commands
    # =========================================================

    def stop_robot(self):

        self.cmd_pub.publish(
            Twist()
        )

    def publish_bucket_action(self, action_name):

        msg = String()
        msg.data = action_name

        self.bucket_action_pub.publish(
            msg
        )

    # =========================================================
    # Action feedback
    # =========================================================

    def publish_feedback(
        self,
        goal_handle,
        state,
        progress,
        detail,
    ):

        feedback = ExecuteRobotTask.Feedback()

        feedback.state = str(state)

        feedback.progress = float(
            max(
                0.0,
                min(
                    1.0,
                    progress,
                ),
            )
        )

        feedback.detail = str(detail)

        goal_handle.publish_feedback(
            feedback
        )

    # =========================================================
    # Action execution
    # =========================================================

    def execute_callback(self, goal_handle):

        self.task_active = True

        request = goal_handle.request

        task_file = request.task_file.strip()

        result = ExecuteRobotTask.Result()

        try:

            # -------------------------------------------------
            # Resolve YAML
            # -------------------------------------------------

            yaml_path = resolve_waypoints_yaml(
                task_file
            )

            if yaml_path is None:

                result.success = False
                result.message = (
                    'Waypoint YAML could not be found: '
                    f'{task_file}'
                )

                self.get_logger().error(
                    result.message
                )

                goal_handle.abort()

                return result

            # -------------------------------------------------
            # Load YAML
            # -------------------------------------------------

            try:

                waypoints = load_waypoints(
                    yaml_path
                )

            except Exception as exc:

                result.success = False
                result.message = (
                    'Failed to load waypoint YAML: '
                    f'{exc}'
                )

                self.get_logger().error(
                    result.message
                )

                goal_handle.abort()

                return result

            if not waypoints:

                result.success = False
                result.message = (
                    'Waypoint YAML contains no valid '
                    f'waypoints: {yaml_path}'
                )

                self.get_logger().error(
                    result.message
                )

                goal_handle.abort()

                return result

            self.get_logger().info(
                f'Executing {len(waypoints)} waypoints '
                f'for {self.truck_name}: {yaml_path}'
            )

            # -------------------------------------------------
            # Wait for fused odometry
            # -------------------------------------------------

            while rclpy.ok() and not self.pose_ready:

                if goal_handle.is_cancel_requested:

                    self.stop_robot()

                    goal_handle.canceled()

                    result.success = False
                    result.message = (
                        'Task canceled while waiting '
                        'for fused odometry.'
                    )

                    return result

                self.publish_feedback(
                    goal_handle,
                    'WAITING_FOR_ODOM',
                    0.0,
                    'Waiting for fused odometry',
                )

                time.sleep(
                    0.1
                )

            # -------------------------------------------------
            # Execute waypoints
            # -------------------------------------------------

            total_waypoints = len(
                waypoints
            )

            current_goal_index = 0

            while (
                rclpy.ok()
                and current_goal_index
                < total_waypoints
            ):

                # ---------------------------------------------
                # Cancellation
                # ---------------------------------------------

                if goal_handle.is_cancel_requested:

                    self.stop_robot()

                    goal_handle.canceled()

                    result.success = False
                    result.message = (
                        f'{self.truck_name} task canceled.'
                    )

                    self.get_logger().warn(
                        result.message
                    )

                    return result

                goal = waypoints[
                    current_goal_index
                ]

                goal_x = goal['x']
                goal_y = goal['y']
                direction = goal['direction']
                waypoint_action = goal['action']

                dx = goal_x - self.x
                dy = goal_y - self.y

                distance = math.sqrt(
                    dx * dx + dy * dy
                )

                # ---------------------------------------------
                # Feedback
                # ---------------------------------------------

                progress = (
                    current_goal_index
                    / total_waypoints
                )

                self.publish_feedback(
                    goal_handle,
                    'NAVIGATING',
                    progress,
                    (
                        f'Waypoint '
                        f'{current_goal_index + 1}/'
                        f'{total_waypoints}, '
                        f'distance={distance:.2f}'
                    ),
                )

                # ---------------------------------------------
                # Waypoint reached
                # ---------------------------------------------

                if distance < self.goal_tolerance:

                    self.stop_robot()

                    self.get_logger().info(
                        'Reached waypoint '
                        f'{current_goal_index + 1}/'
                        f'{total_waypoints}: '
                        f'({goal_x:.2f}, {goal_y:.2f})'
                    )

                    # -----------------------------------------
                    # Optional bucket action
                    # -----------------------------------------

                    if waypoint_action is not None:

                        self.bucket_status = ''
                        self.current_bucket_action = (
                            waypoint_action
                        )
                        self.waiting_for_bucket_action = True

                        self.publish_bucket_action(
                            waypoint_action
                        )

                        self.get_logger().info(
                            'Sent bucket action: '
                            f'{waypoint_action}'
                        )

                        while (
                            rclpy.ok()
                            and self.waiting_for_bucket_action
                        ):

                            if goal_handle.is_cancel_requested:

                                self.stop_robot()

                                goal_handle.canceled()

                                result.success = False
                                result.message = (
                                    'Task canceled while '
                                    'waiting for bucket action.'
                                )

                                return result

                            self.publish_feedback(
                                goal_handle,
                                'PERFORMING_ACTION',
                                progress,
                                (
                                    'Performing bucket action: '
                                    f'{waypoint_action}'
                                ),
                            )

                            if self.bucket_status == 'done':

                                self.get_logger().info(
                                    'Bucket action complete: '
                                    f'{waypoint_action}'
                                )

                                self.waiting_for_bucket_action = False
                                self.current_bucket_action = None

                                break

                            if self.bucket_status == 'failed':

                                self.stop_robot()

                                self.waiting_for_bucket_action = False

                                result.success = False
                                result.message = (
                                    'Bucket action failed: '
                                    f'{waypoint_action}'
                                )

                                self.get_logger().error(
                                    result.message
                                )

                                goal_handle.abort()

                                return result

                            time.sleep(
                                0.1
                            )

                    current_goal_index += 1

                    continue

                # ---------------------------------------------
                # Existing waypoint steering logic
                # ---------------------------------------------

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
                    target_angle
                    - effective_yaw
                )

                cmd = Twist()

                if abs(heading_error) > 0.8:

                    speed = self.slow_speed

                else:

                    speed = self.base_speed

                if abs(heading_error) > 1.2:

                    speed = 0.12

                cmd.linear.x = (
                    direction
                    * speed
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
                    f'pose=('
                    f'{self.x:.2f},'
                    f'{self.y:.2f},'
                    f'{self.yaw:.2f}) '
                    f'dist={distance:.2f} '
                    f'err={heading_error:.2f} '
                    f'dir={direction} '
                    f'action={waypoint_action} '
                    f'cmd_lin={cmd.linear.x:.2f} '
                    f'cmd_ang={cmd.angular.z:.2f}',
                    throttle_duration_sec=0.5,
                )

                time.sleep(
                    0.1
                )

            # -------------------------------------------------
            # Success
            # -------------------------------------------------

            self.stop_robot()

            self.publish_feedback(
                goal_handle,
                'COMPLETE',
                1.0,
                'All waypoints and actions completed',
            )

            goal_handle.succeed()

            result.success = True
            result.message = (
                f'{self.truck_name} successfully completed '
                f'task: {task_file}'
            )

            self.get_logger().info(
                result.message
            )

            return result

        except Exception as exc:

            self.stop_robot()

            result.success = False
            result.message = (
                f'Unhandled task execution error: {exc}'
            )

            self.get_logger().error(
                result.message
            )

            goal_handle.abort()

            return result

        finally:

            self.stop_robot()

            self.task_active = False
            self.waiting_for_bucket_action = False
            self.current_bucket_action = None


def main(args=None):

    rclpy.init(
        args=args
    )

    node = DumpTruckWaypointActionServer()

    executor = MultiThreadedExecutor(
        num_threads=4
    )

    executor.add_node(
        node
    )

    try:

        executor.spin()

    except KeyboardInterrupt:

        pass

    finally:

        node.stop_robot()

        executor.shutdown()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':

    main()