#!/usr/bin/env python3

import os
import time

import yaml

import rclpy
from rclpy.action import (
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.callback_groups import (
    ReentrantCallbackGroup,
)
from rclpy.executors import (
    MultiThreadedExecutor,
)
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from ament_index_python.packages import (
    get_package_share_directory,
)

from construction_site_interfaces.action import (
    ExecuteRobotTask,
)
from construction_site_interfaces.msg import (
    RobotStatus,
)


def resolve_waypoints_yaml(task_file):
    """
    Resolve dump-truck waypoint YAML.

    Search order:
    1. Explicit filesystem path.
    2. Repository operations/dump_truck/waypoints.
    3. Installed dump_truck_control share/waypoints.
    """

    if not task_file:
        return None

    explicit_path = os.path.abspath(
        os.path.expanduser(
            task_file
        )
    )

    if os.path.isfile(
        explicit_path
    ):
        return explicit_path

    search_dir = os.path.dirname(
        os.path.realpath(__file__)
    )

    while True:

        candidate = os.path.join(
            search_dir,
            'operations',
            'dump_truck',
            'waypoints',
            task_file,
        )

        if os.path.isfile(
            candidate
        ):
            return candidate

        parent = os.path.dirname(
            search_dir
        )

        if parent == search_dir:
            break

        search_dir = parent

    try:

        package_share = (
            get_package_share_directory(
                'dump_truck_control'
            )
        )

        installed_path = os.path.join(
            package_share,
            'waypoints',
            task_file,
        )

        if os.path.isfile(
            installed_path
        ):
            return installed_path

    except Exception:
        pass

    return None


def load_waypoints(
    yaml_path,
):
    """
    Load waypoint YAML for mock execution.

    Expected format:

    waypoints:
      - [x, y, direction]
      - [x, y, direction, action]
    """

    with open(
        yaml_path,
        'r',
        encoding='utf-8',
    ) as stream:

        data = yaml.safe_load(
            stream
        ) or {}

    raw_waypoints = data.get(
        'waypoints',
        [],
    )

    if not isinstance(
        raw_waypoints,
        list,
    ):
        raise RuntimeError(
            '"waypoints" must be a list.'
        )

    waypoints = []

    for (
        index,
        waypoint,
    ) in enumerate(
        raw_waypoints
    ):

        if not isinstance(
            waypoint,
            list,
        ):

            raise RuntimeError(
                f'Waypoint {index + 1} '
                'must be a list.'
            )

        if len(
            waypoint
        ) < 3:

            raise RuntimeError(
                f'Waypoint {index + 1} '
                'must contain at least '
                '[x, y, direction].'
            )

        x = float(
            waypoint[0]
        )

        y = float(
            waypoint[1]
        )

        direction = int(
            waypoint[2]
        )

        if direction not in (
            1,
            -1,
        ):

            raise RuntimeError(
                f'Waypoint {index + 1} '
                'direction must be '
                '1 or -1.'
            )

        action = None

        if (
            len(waypoint) >= 4
            and waypoint[3] is not None
        ):

            action = str(
                waypoint[3]
            ).strip()

        waypoints.append(
            {
                'x': x,
                'y': y,
                'direction': direction,
                'action': action,
            }
        )

    if not waypoints:
        raise RuntimeError(
            'Waypoint YAML contains '
            'no valid waypoints.'
        )

    return waypoints


class MockDumpTruckWaypointActionServer(
    Node
):

    def __init__(
        self,
    ):

        super().__init__(
            'mock_waypoint_action_server'
        )

        self.callback_group = (
            ReentrantCallbackGroup()
        )

        # -----------------------------------------------------
        # Parameters
        # -----------------------------------------------------

        self.declare_parameter(
            'truck_name',
            'truck1',
        )

        self.declare_parameter(
            'seconds_per_waypoint',
            0.5,
        )

        self.truck_name = str(
            self.get_parameter(
                'truck_name'
            ).value
        ).strip().strip('/')

        self.seconds_per_waypoint = float(
            self.get_parameter(
                'seconds_per_waypoint'
            ).value
        )

        if (
            self.seconds_per_waypoint
            <= 0.0
        ):

            raise RuntimeError(
                'seconds_per_waypoint '
                'must be greater than zero.'
            )

        # -----------------------------------------------------
        # ROS names
        # -----------------------------------------------------

        self.action_name = (
            f'/{self.truck_name}/'
            'execute_robot_task'
        )

        self.status_topic = (
            f'/{self.truck_name}/status'
        )

        # -----------------------------------------------------
        # Status publisher
        # -----------------------------------------------------

        status_qos = QoSProfile(
            history=(
                HistoryPolicy.KEEP_LAST
            ),
            depth=1,
            reliability=(
                ReliabilityPolicy.RELIABLE
            ),
            durability=(
                DurabilityPolicy.TRANSIENT_LOCAL
            ),
        )

        self.status_pub = (
            self.create_publisher(
                RobotStatus,
                self.status_topic,
                status_qos,
            )
        )

        self.current_status_state = (
            'idle'
        )

        self.current_status_detail = (
            'Mock server waiting for task'
        )

        # -----------------------------------------------------
        # Action state
        # -----------------------------------------------------

        self.task_active = False

        # -----------------------------------------------------
        # Action server
        # -----------------------------------------------------

        self.action_server = (
            ActionServer(
                self,
                ExecuteRobotTask,
                self.action_name,
                execute_callback=(
                    self.execute_callback
                ),
                goal_callback=(
                    self.goal_callback
                ),
                cancel_callback=(
                    self.cancel_callback
                ),
                callback_group=(
                    self.callback_group
                ),
            )
        )

        # -----------------------------------------------------
        # Status heartbeat
        # -----------------------------------------------------

        self.status_timer = (
            self.create_timer(
                1.0,
                self.republish_status,
                callback_group=(
                    self.callback_group
                ),
            )
        )

        self.publish_status(
            'idle',
            'Mock server waiting for task',
        )

        self.get_logger().info(
            'Mock Dump Truck Action '
            'Server started. '
            f'truck={self.truck_name}, '
            f'action={self.action_name}, '
            f'status={self.status_topic}, '
            'seconds_per_waypoint='
            f'{self.seconds_per_waypoint:.2f}'
        )

    # =========================================================
    # Status
    # =========================================================

    def publish_status(
        self,
        state,
        detail,
    ):

        self.current_status_state = (
            str(state)
            .strip()
            .lower()
        )

        self.current_status_detail = (
            str(detail)
        )

        msg = RobotStatus()

        msg.robot_name = (
            self.truck_name
        )

        msg.state = (
            self.current_status_state
        )

        msg.detail = (
            self.current_status_detail
        )

        self.status_pub.publish(
            msg
        )

        self.get_logger().info(
            'STATUS: '
            f'robot={msg.robot_name}, '
            f'state={msg.state}, '
            f'detail={msg.detail}'
        )

    def republish_status(
        self,
    ):

        if not rclpy.ok():
            return

        msg = RobotStatus()

        msg.robot_name = (
            self.truck_name
        )

        msg.state = (
            self.current_status_state
        )

        msg.detail = (
            self.current_status_detail
        )

        self.status_pub.publish(
            msg
        )

    # =========================================================
    # Action callbacks
    # =========================================================

    def goal_callback(
        self,
        goal_request,
    ):

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

        if (
            requested_robot
            != self.truck_name
        ):

            self.get_logger().warn(
                'Rejecting goal: '
                f'requested robot='
                f'{requested_robot}, '
                f'this server='
                f'{self.truck_name}'
            )

            return GoalResponse.REJECT

        if task_type != 'waypoint':

            self.get_logger().warn(
                'Rejecting unsupported '
                'task type: '
                f'{task_type}'
            )

            return GoalResponse.REJECT

        if self.task_active:

            self.get_logger().warn(
                'Rejecting goal because '
                'another mock task '
                'is already active.'
            )

            return GoalResponse.REJECT

        self.get_logger().info(
            'Accepted mock task request: '
            f'robot={requested_robot}, '
            f'type={task_type}, '
            f'file={goal_request.task_file}'
        )

        return GoalResponse.ACCEPT

    def cancel_callback(
        self,
        goal_handle,
    ):

        self.get_logger().warn(
            'Cancel requested for '
            f'{self.truck_name}'
        )

        return CancelResponse.ACCEPT

    # =========================================================
    # Feedback
    # =========================================================

    def publish_feedback(
        self,
        goal_handle,
        state,
        progress,
        detail,
    ):

        feedback = (
            ExecuteRobotTask.Feedback()
        )

        feedback.state = str(
            state
        )

        feedback.progress = float(
            max(
                0.0,
                min(
                    1.0,
                    progress,
                ),
            )
        )

        feedback.detail = str(
            detail
        )

        goal_handle.publish_feedback(
            feedback
        )

    # =========================================================
    # Action execution
    # =========================================================

    def execute_callback(
        self,
        goal_handle,
    ):

        self.task_active = True

        request = (
            goal_handle.request
        )

        task_file = (
            request.task_file.strip()
        )

        result = (
            ExecuteRobotTask.Result()
        )

        try:

            # -------------------------------------------------
            # Resolve YAML
            # -------------------------------------------------

            yaml_path = (
                resolve_waypoints_yaml(
                    task_file
                )
            )

            if yaml_path is None:

                result.success = False

                result.message = (
                    'Waypoint YAML could not '
                    'be found: '
                    f'{task_file}'
                )

                self.publish_status(
                    'fault',
                    result.message,
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

                waypoints = (
                    load_waypoints(
                        yaml_path
                    )
                )

            except Exception as exc:

                result.success = False

                result.message = (
                    'Failed to load waypoint '
                    'YAML: '
                    f'{exc}'
                )

                self.publish_status(
                    'fault',
                    result.message,
                )

                self.get_logger().error(
                    result.message
                )

                goal_handle.abort()

                return result

            total_waypoints = len(
                waypoints
            )

            self.get_logger().info(
                'MOCK EXECUTION START: '
                f'truck={self.truck_name}, '
                f'file={task_file}, '
                f'waypoints={total_waypoints}'
            )

            self.publish_status(
                'navigating',
                (
                    'Mock execution of '
                    f'{task_file}'
                ),
            )

            # -------------------------------------------------
            # Simulated waypoint execution
            # -------------------------------------------------

            for (
                index,
                waypoint,
            ) in enumerate(
                waypoints
            ):

                waypoint_number = (
                    index + 1
                )

                if (
                    goal_handle
                    .is_cancel_requested
                ):

                    goal_handle.canceled()

                    result.success = False

                    result.message = (
                        f'{self.truck_name} '
                        'mock task canceled.'
                    )

                    self.publish_status(
                        'idle',
                        result.message,
                    )

                    return result

                progress = (
                    index
                    / total_waypoints
                )

                waypoint_detail = (
                    'Mock waypoint '
                    f'{waypoint_number}/'
                    f'{total_waypoints}: '
                    f'x={waypoint["x"]:.2f}, '
                    f'y={waypoint["y"]:.2f}, '
                    'direction='
                    f'{waypoint["direction"]}'
                )

                if (
                    waypoint['action']
                    is not None
                ):

                    waypoint_detail += (
                        ', action='
                        f'{waypoint["action"]}'
                    )

                self.publish_feedback(
                    goal_handle,
                    'MOCK_NAVIGATING',
                    progress,
                    waypoint_detail,
                )

                self.get_logger().info(
                    waypoint_detail
                )

                # ---------------------------------------------
                # Wait while remaining cancel-responsive
                # ---------------------------------------------

                start_time = (
                    time.monotonic()
                )

                while rclpy.ok():

                    if (
                        goal_handle
                        .is_cancel_requested
                    ):

                        goal_handle.canceled()

                        result.success = False

                        result.message = (
                            f'{self.truck_name} '
                            'mock task canceled.'
                        )

                        self.publish_status(
                            'idle',
                            result.message,
                        )

                        return result

                    elapsed = (
                        time.monotonic()
                        - start_time
                    )

                    if (
                        elapsed
                        >= self.seconds_per_waypoint
                    ):

                        break

                    time.sleep(
                        0.05
                    )

            # -------------------------------------------------
            # Success
            # -------------------------------------------------

            self.publish_feedback(
                goal_handle,
                'COMPLETE',
                1.0,
                (
                    'Mock waypoint task '
                    'completed'
                ),
            )

            goal_handle.succeed()

            result.success = True

            result.message = (
                f'{self.truck_name} '
                'successfully completed '
                'mock waypoint task: '
                f'{task_file}'
            )

            self.publish_status(
                'completed',
                result.message,
            )

            self.get_logger().info(
                'MOCK EXECUTION COMPLETE: '
                f'{result.message}'
            )

            return result

        except Exception as exc:

            result.success = False

            result.message = (
                'Unhandled mock task '
                'execution error: '
                f'{exc}'
            )

            self.publish_status(
                'fault',
                result.message,
            )

            self.get_logger().error(
                result.message
            )

            goal_handle.abort()

            return result

        finally:

            self.task_active = False


def main(
    args=None,
):

    rclpy.init(
        args=args
    )

    node = (
        MockDumpTruckWaypointActionServer()
    )

    executor = (
        MultiThreadedExecutor(
            num_threads=4
        )
    )

    executor.add_node(
        node
    )

    try:

        executor.spin()

    except KeyboardInterrupt:

        pass

    finally:

        executor.shutdown()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()