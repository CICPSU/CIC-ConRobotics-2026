import math
from typing import Dict, Sequence, Tuple


class ExcavatorGoalValidationError(ValueError):
    """Raised when a FollowJointTrajectory goal is invalid."""


JointLimits = Dict[str, Tuple[float, float]]


def validate_joint_targets(
    joint_names: Sequence[str],
    points,
    joint_limits_rad: JointLimits,
) -> None:
    """
    Validate trajectory joint names and target positions.

    Parameters
    ----------
    joint_names:
        Joint names supplied by the FollowJointTrajectory goal.

    points:
        Trajectory points. Each point must provide one position for
        each joint name.

    joint_limits_rad:
        Mapping:

            joint_name -> (minimum_angle_rad, maximum_angle_rad)

    Raises
    ------
    ExcavatorGoalValidationError
        If the trajectory is structurally invalid or any target lies
        outside the configured joint limits.
    """

    names = list(joint_names)

    if not names:
        raise ExcavatorGoalValidationError(
            "Trajectory does not contain any joint names"
        )

    if not points:
        raise ExcavatorGoalValidationError(
            "Trajectory does not contain any points"
        )

    if len(set(names)) != len(names):
        raise ExcavatorGoalValidationError(
            f"Duplicate joint names are not allowed: {names}"
        )

    unknown = [
        joint_name
        for joint_name in names
        if joint_name not in joint_limits_rad
    ]

    if unknown:
        raise ExcavatorGoalValidationError(
            f"Unknown joints: {unknown}"
        )

    number_of_joints = len(names)

    for point_index, point in enumerate(points):
        if len(point.positions) != number_of_joints:
            raise ExcavatorGoalValidationError(
                f"Point {point_index} has "
                f"{len(point.positions)} positions for "
                f"{number_of_joints} joints"
            )

        for joint_index, joint_name in enumerate(names):
            target = float(point.positions[joint_index])

            if not math.isfinite(target):
                raise ExcavatorGoalValidationError(
                    f"Point {point_index} "
                    f"{joint_name} target is not finite: {target}"
                )

            lower, upper = joint_limits_rad[joint_name]

            if target < lower or target > upper:
                raise ExcavatorGoalValidationError(
                    f"Point {point_index} "
                    f"{joint_name} target "
                    f"{math.degrees(target):+.2f} deg "
                    f"is outside configured range "
                    f"[{math.degrees(lower):+.2f}, "
                    f"{math.degrees(upper):+.2f}] deg"
                )