#!/usr/bin/env python3

import argparse
import math
import sys
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

from excavator_control.trajectory_loader import (
    ExcavatorTrajectoryError,
    load_excavator_trajectory,
)


DEFAULT_ACTION_NAME = (
    "/upper_arm_controller/follow_joint_trajectory"
)

JOINT_NAMES = [
    "swing_joint",
    "boom_joint",
    "arm_joint",
    "bucket_joint",
]


def seconds_to_duration(seconds: float):
    """
    Convert floating-point seconds to ROS Duration message fields.

    Returns
    -------
    tuple[int, int]
        seconds and nanoseconds.
    """

    whole_seconds = int(seconds)
    nanoseconds = int(
        round(
            (seconds - whole_seconds) * 1_000_000_000
        )
    )

    if nanoseconds >= 1_000_000_000:
        whole_seconds += 1
        nanoseconds -= 1_000_000_000

    return whole_seconds, nanoseconds


class ExcavatorTrajectoryClient(Node):
    """
    ROS 2 Action client for sending excavator trajectories.

    The trajectory itself is loaded from an operational YAML file.
    """

    def __init__(
        self,
        action_name: str,
    ):
        super().__init__(
            "excavator_trajectory_client"
        )

        self._action_name = action_name

        self._client = ActionClient(
            self,
            FollowJointTrajectory,
            self._action_name,
        )

    def send_trajectory(
        self,
        trajectory,
        seconds_per_waypoint: float,
    ):
        """
        Convert a loaded ExcavatorTrajectory into a ROS Action goal.
        """

        goal = FollowJointTrajectory.Goal()

        goal.trajectory.joint_names = list(
            JOINT_NAMES
        )

        for index, waypoint in enumerate(
            trajectory.waypoints
        ):
            point = JointTrajectoryPoint()

            point.positions = [
                math.radians(
                    waypoint.swing_deg
                ),
                math.radians(
                    waypoint.boom_deg
                ),
                math.radians(
                    waypoint.arm_deg
                ),
                math.radians(
                    waypoint.bucket_deg
                ),
            ]

            # The first waypoint occurs after one interval.
            #
            # Example with 3 s/waypoint:
            #
            # waypoint 1 -> 3 s
            # waypoint 2 -> 6 s
            # waypoint 3 -> 9 s
            #
            time_from_start = (
                (index + 1)
                * seconds_per_waypoint
            )

            sec, nanosec = seconds_to_duration(
                time_from_start
            )

            point.time_from_start.sec = sec
            point.time_from_start.nanosec = (
                nanosec
            )

            goal.trajectory.points.append(
                point
            )

        self.get_logger().info(
            "Waiting for excavator trajectory "
            f"server: {self._action_name}"
        )

        if not self._client.wait_for_server(
            timeout_sec=10.0
        ):
            self.get_logger().error(
                "Trajectory server was not found "
                "within 10 seconds."
            )
            return False

        self.get_logger().info(
            "Sending trajectory "
            f"'{trajectory.trajectory_name}' "
            f"with {len(trajectory.waypoints)} "
            "waypoints."
        )

        for index, waypoint in enumerate(
            trajectory.waypoints,
            start=1,
        ):
            self.get_logger().info(
                f"  {index:02d}. "
                f"{waypoint.name}: "
                f"swing={waypoint.swing_deg:.1f} deg, "
                f"boom={waypoint.boom_deg:.1f} deg, "
                f"arm={waypoint.arm_deg:.1f} deg, "
                f"bucket={waypoint.bucket_deg:.1f} deg"
            )

        send_goal_future = (
            self._client.send_goal_async(
                goal,
                feedback_callback=self._feedback_callback,
            )
        )

        rclpy.spin_until_future_complete(
            self,
            send_goal_future,
        )

        goal_handle = send_goal_future.result()

        if goal_handle is None:
            self.get_logger().error(
                "Failed to receive a response "
                "from the Action server."
            )
            return False

        if not goal_handle.accepted:
            self.get_logger().error(
                "Trajectory goal was REJECTED."
            )
            return False

        self.get_logger().info(
            "Trajectory goal ACCEPTED."
        )

        result_future = (
            goal_handle.get_result_async()
        )

        rclpy.spin_until_future_complete(
            self,
            result_future,
        )

        wrapped_result = result_future.result()

        if wrapped_result is None:
            self.get_logger().error(
                "No trajectory result received."
            )
            return False

        status = wrapped_result.status
        result = wrapped_result.result

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                "Trajectory completed SUCCESSFULLY."
            )

            if result.error_string:
                self.get_logger().info(
                    f"Server message: "
                    f"{result.error_string}"
                )

            return True

        if status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warning(
                "Trajectory was CANCELED."
            )

        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error(
                "Trajectory was ABORTED."
            )

        else:
            self.get_logger().error(
                "Trajectory finished with ROS "
                f"Action status {status}."
            )

        self.get_logger().error(
            "FollowJointTrajectory result: "
            f"error_code={result.error_code}, "
            f"error_string='{result.error_string}'"
        )

        return False

    def _feedback_callback(
        self,
        feedback_msg,
    ):
        feedback = feedback_msg.feedback

        actual = list(
            feedback.actual.positions
        )

        desired = list(
            feedback.desired.positions
        )

        if not actual:
            return

        actual_deg = [
            math.degrees(value)
            for value in actual
        ]

        desired_deg = [
            math.degrees(value)
            for value in desired
        ]

        actual_text = ", ".join(
            f"{value:.1f}"
            for value in actual_deg
        )

        desired_text = ", ".join(
            f"{value:.1f}"
            for value in desired_deg
        )

        self.get_logger().info(
            "Feedback [deg] "
            f"desired=[{desired_text}] "
            f"actual=[{actual_text}]"
        )


def parse_arguments():
    parser = argparse.ArgumentParser(
        description=(
            "Send an excavator trajectory YAML "
            "to the FollowJointTrajectory server."
        )
    )

    parser.add_argument(
        "trajectory_file",
        type=Path,
        help=(
            "Path to an excavator trajectory "
            "YAML file."
        ),
    )

    parser.add_argument(
        "--seconds-per-waypoint",
        type=float,
        default=3.0,
        help=(
            "Time allocated to each waypoint "
            "(default: 3.0 seconds)."
        ),
    )

    parser.add_argument(
        "--action-name",
        default=DEFAULT_ACTION_NAME,
        help=(
            "FollowJointTrajectory action name."
        ),
    )

    return parser.parse_args()


def main(args=None):
    cli_args = parse_arguments()

    if cli_args.seconds_per_waypoint <= 0.0:
        print(
            "ERROR: --seconds-per-waypoint "
            "must be greater than zero.",
            file=sys.stderr,
        )
        return 1

    try:
        trajectory = (
            load_excavator_trajectory(
                cli_args.trajectory_file
            )
        )

    except ExcavatorTrajectoryError as exc:
        print(
            "ERROR: Invalid excavator trajectory:",
            file=sys.stderr,
        )
        print(
            f"  {exc}",
            file=sys.stderr,
        )
        return 1

    rclpy.init(args=args)

    node = ExcavatorTrajectoryClient(
        action_name=cli_args.action_name
    )

    try:
        success = node.send_trajectory(
            trajectory=trajectory,
            seconds_per_waypoint=(
                cli_args.seconds_per_waypoint
            ),
        )

    except KeyboardInterrupt:
        node.get_logger().warning(
            "Trajectory client interrupted."
        )
        success = False

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())