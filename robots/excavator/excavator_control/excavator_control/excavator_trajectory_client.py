import argparse
import math
import sys
from typing import Dict, List

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


ROS_JOINT_NAMES: Dict[str, str] = {
    "swing": "swing_joint",
    "boom": "boom_joint",
    "arm": "arm_joint",
    "bucket": "bucket_joint",
}


def seconds_to_duration(
    seconds: float,
):
    sec = int(seconds)

    nanosec = int(
        round(
            (seconds - sec)
            * 1_000_000_000
        )
    )

    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000

    return sec, nanosec


class ExcavatorTrajectoryClient(Node):
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

    def _feedback_callback(
        self,
        feedback_msg,
    ):
        feedback = feedback_msg.feedback

        desired = list(
            feedback.desired.positions
        )
        actual = list(
            feedback.actual.positions
        )
        error = list(
            feedback.error.positions
        )

        desired_deg = [
            round(
                math.degrees(value),
                1,
            )
            for value in desired
        ]

        actual_deg = [
            round(
                math.degrees(value),
                1,
            )
            for value in actual
        ]

        error_deg = [
            round(
                math.degrees(value),
                1,
            )
            for value in error
        ]

        self.get_logger().info(
            "Feedback [deg] "
            f"desired={desired_deg} "
            f"actual={actual_deg} "
            f"error={error_deg}"
        )

    def send_trajectory(
        self,
        trajectory,
        seconds_per_waypoint: float,
    ) -> bool:
        goal = FollowJointTrajectory.Goal()

        ros_joint_names: List[str] = [
            ROS_JOINT_NAMES[joint]
            for joint in trajectory.joints
        ]

        goal.trajectory.joint_names = (
            ros_joint_names
        )

        for index, waypoint in enumerate(
            trajectory.waypoints
        ):
            point = JointTrajectoryPoint()

            point.positions = [
                math.radians(
                    waypoint.positions_deg[
                        joint
                    ]
                )
                for joint in trajectory.joints
            ]

            if "swing" in trajectory.joints:
                point.velocities = [
                    float(waypoint.swing_direction) if joint == "swing" else 0.0
                    for joint in trajectory.joints
                ]

            time_from_start = (
                (index + 1)
                * seconds_per_waypoint
            )

            sec, nanosec = (
                seconds_to_duration(
                    time_from_start
                )
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
            f"Sending trajectory "
            f"'{trajectory.trajectory_name}' "
            f"with {len(trajectory.waypoints)} "
            "waypoints."
        )

        self.get_logger().info(
            "Commanded joints: "
            + ", ".join(
                ros_joint_names
            )
        )

        for index, waypoint in enumerate(
            trajectory.waypoints,
            start=1,
        ):
            position_text = ", ".join(
                (
                    f"{joint}="
                    f"{waypoint.positions_deg[joint]:.1f} deg"
                )
                for joint in trajectory.joints
            )

            self.get_logger().info(
                f"  {index:02d}. "
                f"{waypoint.name}: "
                f"{position_text}"
            )

        send_goal_future = (
            self._client.send_goal_async(
                goal,
                feedback_callback=(
                    self._feedback_callback
                ),
            )
        )

        rclpy.spin_until_future_complete(
            self,
            send_goal_future,
        )

        goal_handle = (
            send_goal_future.result()
        )

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

        wrapped_result = (
            result_future.result()
        )

        if wrapped_result is None:
            self.get_logger().error(
                "No trajectory result received."
            )
            return False

        status = wrapped_result.status
        result = wrapped_result.result

        if (
            status
            == GoalStatus.STATUS_SUCCEEDED
        ):
            self.get_logger().info(
                "Trajectory completed "
                "SUCCESSFULLY."
            )

            self.get_logger().info(
                "Server message: "
                f"{result.error_string}"
            )

            return True

        self.get_logger().error(
            "Trajectory did not complete "
            "successfully."
        )

        self.get_logger().error(
            f"Status: {status}"
        )

        self.get_logger().error(
            f"Error code: "
            f"{result.error_code}"
        )

        self.get_logger().error(
            "Server message: "
            f"{result.error_string}"
        )

        return False


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Send an excavator trajectory YAML "
            "to the FollowJointTrajectory "
            "Action server."
        )
    )

    parser.add_argument(
        "trajectory",
        help=(
            "Path to an excavator trajectory "
            "YAML file"
        ),
    )

    parser.add_argument(
        "--seconds-per-waypoint",
        type=float,
        default=3.0,
        help=(
            "Time between trajectory waypoints "
            "in seconds. Default: 3.0"
        ),
    )

    parser.add_argument(
        "--action-name",
        default=DEFAULT_ACTION_NAME,
        help=(
            "FollowJointTrajectory Action name"
        ),
    )

    return parser.parse_args()


def main(args=None):
    cli_args = parse_args()

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
                cli_args.trajectory
            )
        )
    except ExcavatorTrajectoryError as exc:
        print(
            f"ERROR: {exc}",
            file=sys.stderr,
        )
        return 1

    rclpy.init(
        args=args
    )

    node = ExcavatorTrajectoryClient(
        action_name=cli_args.action_name
    )

    success = False

    try:
        success = node.send_trajectory(
            trajectory=trajectory,
            seconds_per_waypoint=(
                cli_args.seconds_per_waypoint
            ),
        )

    except KeyboardInterrupt:
        node.get_logger().warn(
            "Trajectory client interrupted."
        )

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(
        main()
    )
