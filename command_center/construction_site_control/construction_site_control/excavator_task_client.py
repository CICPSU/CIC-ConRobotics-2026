#!/usr/bin/env python3

import argparse
import math
import os
import sys

import yaml

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


VALID_JOINTS = (
    'swing',
    'boom',
    'arm',
    'bucket',
)

ROS_JOINT_NAMES = {
    'swing': 'swing_joint',
    'boom': 'boom_joint',
    'arm': 'arm_joint',
    'bucket': 'bucket_joint',
}


class ExcavatorTaskError(RuntimeError):
    """Raised when an excavator task cannot be executed."""


def normalize_robot_name(robot_name):
    """
    Normalize a robot name for use in ROS names.

    Examples:
        excavator3   -> excavator3
        /excavator3 -> excavator3
    """

    normalized = str(
        robot_name
    ).strip().strip('/')

    if not normalized:
        raise ExcavatorTaskError(
            'Robot name cannot be empty.'
        )

    return normalized


def get_excavator_action_name(
    robot_name,
):
    """
    Build the namespaced FollowJointTrajectory
    Action name for an excavator.

    Example:
        excavator3
        ->
        /excavator3/upper_arm_controller/
        follow_joint_trajectory
    """

    robot_name = normalize_robot_name(
        robot_name
    )

    return (
        f'/{robot_name}/'
        'upper_arm_controller/'
        'follow_joint_trajectory'
    )


def resolve_trajectory_yaml(
    trajectory_file,
):
    """
    Resolve an excavator trajectory YAML.

    Search order:
    1. Explicit filesystem path.
    2. Repository operations/excavator/trajectories.
    """

    if not trajectory_file:
        return None

    explicit_path = os.path.abspath(
        os.path.expanduser(
            trajectory_file
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
            'excavator',
            'trajectories',
            trajectory_file,
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

    return None


def load_trajectory(
    trajectory_path,
):
    """
    Load an excavator trajectory YAML.

    This loader intentionally keeps the Command Center
    independent from the excavator_control Python package.

    The excavator Action Server remains responsible for
    final machine-specific goal validation.
    """

    with open(
        trajectory_path,
        'r',
        encoding='utf-8',
    ) as stream:
        data = yaml.safe_load(
            stream
        ) or {}

    trajectory_name = str(
        data.get(
            'trajectory_name',
            os.path.basename(
                trajectory_path
            ),
        )
    )

    joints = data.get(
        'joints',
        [],
    )

    waypoints = data.get(
        'waypoints',
        [],
    )

    if not isinstance(
        joints,
        list,
    ) or not joints:
        raise ExcavatorTaskError(
            'Trajectory must contain '
            'a non-empty "joints" list.'
        )

    if not isinstance(
        waypoints,
        list,
    ) or not waypoints:
        raise ExcavatorTaskError(
            'Trajectory must contain '
            'a non-empty "waypoints" list.'
        )

    normalized_joints = []

    for joint in joints:
        joint_name = str(
            joint
        ).strip()

        if joint_name not in VALID_JOINTS:
            raise ExcavatorTaskError(
                f'Unknown excavator joint: '
                f'{joint_name}'
            )

        if joint_name in normalized_joints:
            raise ExcavatorTaskError(
                f'Duplicate excavator joint: '
                f'{joint_name}'
            )

        normalized_joints.append(
            joint_name
        )

    normalized_waypoints = []

    for index, waypoint in enumerate(
        waypoints
    ):
        if not isinstance(
            waypoint,
            dict,
        ):
            raise ExcavatorTaskError(
                f'Waypoint {index} must be '
                'a mapping.'
            )

        waypoint_name = str(
            waypoint.get(
                'name',
                f'waypoint_{index + 1}',
            )
        )

        positions = waypoint.get(
            'positions',
            {},
        )

        if not isinstance(
            positions,
            dict,
        ):
            raise ExcavatorTaskError(
                f'Waypoint "{waypoint_name}" '
                'positions must be a mapping.'
            )

        position_values = []

        for joint in normalized_joints:
            if joint not in positions:
                raise ExcavatorTaskError(
                    f'Waypoint "{waypoint_name}" '
                    f'is missing joint "{joint}".'
                )

            try:
                position = float(
                    positions[joint]
                )
            except (
                TypeError,
                ValueError,
            ) as exc:
                raise ExcavatorTaskError(
                    f'Waypoint "{waypoint_name}" '
                    f'joint "{joint}" is not numeric.'
                ) from exc

            if not math.isfinite(
                position
            ):
                raise ExcavatorTaskError(
                    f'Waypoint "{waypoint_name}" '
                    f'joint "{joint}" is not finite.'
                )

            position_values.append(
                position
            )

        extra_joints = (
            set(positions.keys())
            - set(normalized_joints)
        )

        if extra_joints:
            raise ExcavatorTaskError(
                f'Waypoint "{waypoint_name}" '
                'contains joints not listed '
                'in trajectory joints: '
                f'{sorted(extra_joints)}'
            )

        normalized_waypoints.append(
            {
                'name': waypoint_name,
                'positions_deg': (
                    position_values
                ),
            }
        )

    return {
        'trajectory_name': (
            trajectory_name
        ),
        'joints': normalized_joints,
        'waypoints': normalized_waypoints,
    }


def seconds_to_duration(
    seconds,
):
    sec = int(
        seconds
    )

    nanosec = int(
        round(
            (
                seconds
                - sec
            )
            * 1_000_000_000
        )
    )

    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000

    return sec, nanosec


class ExcavatorTaskClient(Node):

    def __init__(
        self,
        robot_name,
        action_name=None,
    ):
        self.robot_name = normalize_robot_name(
            robot_name
        )

        node_name = (
            f'{self.robot_name}_task_client'
        )

        super().__init__(
            node_name
        )

        if action_name:
            self.action_name = str(
                action_name
            ).strip()
        else:
            self.action_name = (
                get_excavator_action_name(
                    self.robot_name
                )
            )

        if not self.action_name:
            raise ExcavatorTaskError(
                'Excavator Action name '
                'cannot be empty.'
            )

        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.action_name,
        )

        self.get_logger().info(
            'Excavator Task Client initialized.'
        )

        self.get_logger().info(
            f'  Robot  : {self.robot_name}'
        )

        self.get_logger().info(
            f'  Action : {self.action_name}'
        )

    def feedback_callback(
        self,
        feedback_msg,
    ):
        feedback = (
            feedback_msg.feedback
        )

        actual_deg = [
            round(
                math.degrees(
                    value
                ),
                1,
            )
            for value in (
                feedback.actual.positions
            )
        ]

        desired_deg = [
            round(
                math.degrees(
                    value
                ),
                1,
            )
            for value in (
                feedback.desired.positions
            )
        ]

        self.get_logger().info(
            f'{self.robot_name} feedback: '
            f'desired={desired_deg} deg, '
            f'actual={actual_deg} deg'
        )

    def execute_trajectory(
        self,
        trajectory,
        seconds_per_waypoint,
    ):
        if seconds_per_waypoint <= 0.0:
            raise ExcavatorTaskError(
                'seconds_per_waypoint must '
                'be greater than zero.'
            )

        goal = (
            FollowJointTrajectory.Goal()
        )

        goal.trajectory.joint_names = [
            ROS_JOINT_NAMES[joint]
            for joint in (
                trajectory['joints']
            )
        ]

        for index, waypoint in enumerate(
            trajectory['waypoints']
        ):
            point = (
                JointTrajectoryPoint()
            )

            point.positions = [
                math.radians(
                    value
                )
                for value in (
                    waypoint[
                        'positions_deg'
                    ]
                )
            ]

            waypoint_time = (
                (index + 1)
                * seconds_per_waypoint
            )

            sec, nanosec = (
                seconds_to_duration(
                    waypoint_time
                )
            )

            point.time_from_start.sec = (
                sec
            )

            point.time_from_start.nanosec = (
                nanosec
            )

            goal.trajectory.points.append(
                point
            )

        self.get_logger().info(
            'Waiting for excavator '
            f'Action server: '
            f'{self.action_name}'
        )

        if not (
            self.action_client.wait_for_server(
                timeout_sec=10.0
            )
        ):
            raise ExcavatorTaskError(
                'Excavator Action server '
                'was not found within '
                '10 seconds: '
                f'{self.action_name}'
            )

        self.get_logger().info(
            'Sending excavator trajectory '
            f'"{trajectory["trajectory_name"]}" '
            f'to {self.robot_name}'
        )

        self.get_logger().info(
            'Joints: '
            + ', '.join(
                trajectory['joints']
            )
        )

        future = (
            self.action_client.send_goal_async(
                goal,
                feedback_callback=(
                    self.feedback_callback
                ),
            )
        )

        rclpy.spin_until_future_complete(
            self,
            future,
        )

        goal_handle = (
            future.result()
        )

        if goal_handle is None:
            raise ExcavatorTaskError(
                'No response received from '
                'the excavator Action server.'
            )

        if not goal_handle.accepted:
            self.get_logger().error(
                'Excavator trajectory '
                'was REJECTED.'
            )
            return False

        self.get_logger().info(
            'Excavator trajectory '
            'ACCEPTED.'
        )

        result_future = (
            goal_handle.get_result_async()
        )

        rclpy.spin_until_future_complete(
            self,
            result_future,
        )

        wrapped_result = (
            result_future.result()
        )

        if wrapped_result is None:
            raise ExcavatorTaskError(
                'No result received from '
                'the excavator Action server.'
            )

        result = (
            wrapped_result.result
        )

        if (
            wrapped_result.status
            == GoalStatus.STATUS_SUCCEEDED
        ):
            self.get_logger().info(
                'Excavator trajectory '
                'completed SUCCESSFULLY.'
            )

            if result.error_string:
                self.get_logger().info(
                    'Server message: '
                    f'{result.error_string}'
                )

            return True

        self.get_logger().error(
            'Excavator trajectory FAILED. '
            f'status={wrapped_result.status}, '
            f'error_code={result.error_code}, '
            f'message={result.error_string}'
        )

        return False


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            'Send an excavator trajectory '
            'from the Construction Robotics '
            'Command Center.'
        )
    )

    parser.add_argument(
        'trajectory',
        help=(
            'Excavator trajectory YAML '
            'filename or path.'
        ),
    )

    parser.add_argument(
        '--robot',
        default='excavator1',
        help=(
            'Excavator robot name. '
            'The Action name is generated '
            'automatically from this value. '
            'Example: excavator3. '
            'Default: excavator1.'
        ),
    )

    parser.add_argument(
        '--seconds-per-waypoint',
        type=float,
        default=3.0,
        help=(
            'Time between trajectory '
            'waypoints. Default: 3.0 seconds.'
        ),
    )

    parser.add_argument(
        '--action-name',
        default=None,
        help=(
            'Optional explicit '
            'FollowJointTrajectory Action name. '
            'If omitted, the Action name is '
            'generated as '
            '/<robot>/upper_arm_controller/'
            'follow_joint_trajectory.'
        ),
    )

    return parser.parse_args()


def main(args=None):
    cli_args = (
        parse_args()
    )

    trajectory_path = (
        resolve_trajectory_yaml(
            cli_args.trajectory
        )
    )

    if trajectory_path is None:
        print(
            'ERROR: Excavator trajectory '
            'could not be found: '
            f'{cli_args.trajectory}',
            file=sys.stderr,
        )
        return 1

    try:
        trajectory = load_trajectory(
            trajectory_path
        )

        robot_name = normalize_robot_name(
            cli_args.robot
        )

    except (
        OSError,
        yaml.YAMLError,
        ExcavatorTaskError,
    ) as exc:
        print(
            f'ERROR: {exc}',
            file=sys.stderr,
        )
        return 1

    rclpy.init(
        args=args
    )

    node = None
    success = False

    try:
        node = ExcavatorTaskClient(
            robot_name=robot_name,
            action_name=(
                cli_args.action_name
            ),
        )

        success = (
            node.execute_trajectory(
                trajectory=trajectory,
                seconds_per_waypoint=(
                    cli_args.seconds_per_waypoint
                ),
            )
        )

    except ExcavatorTaskError as exc:
        if node is not None:
            node.get_logger().error(
                str(exc)
            )
        else:
            print(
                f'ERROR: {exc}',
                file=sys.stderr,
            )

    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().warning(
                'Excavator task interrupted.'
            )

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    return (
        0
        if success
        else 1
    )


if __name__ == '__main__':
    raise SystemExit(
        main()
    )