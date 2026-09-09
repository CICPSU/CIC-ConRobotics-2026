import math

import pytest
from trajectory_msgs.msg import JointTrajectoryPoint

from excavator_control.goal_validation import (
    ExcavatorGoalValidationError,
    validate_joint_targets,
)


JOINT_LIMITS_RAD = {
    "swing_joint": (
        math.radians(-90.0),
        math.radians(90.0),
    ),
    "boom_joint": (
        math.radians(-60.0),
        math.radians(5.0),
    ),
    "arm_joint": (
        math.radians(52.0),
        math.radians(112.0),
    ),
    "bucket_joint": (
        math.radians(0.0),
        math.radians(90.0),
    ),
}


def make_point(*positions):
    point = JointTrajectoryPoint()
    point.positions = list(positions)
    return point


def test_valid_goal_passes():
    names = [
        "swing_joint",
        "boom_joint",
        "arm_joint",
        "bucket_joint",
    ]

    points = [
        make_point(
            math.radians(0.0),
            math.radians(-20.0),
            math.radians(80.0),
            math.radians(30.0),
        )
    ]

    validate_joint_targets(
        names,
        points,
        JOINT_LIMITS_RAD,
    )


def test_rejects_boom_above_maximum():
    names = ["boom_joint"]

    points = [
        make_point(
            math.radians(20.0),
        )
    ]

    with pytest.raises(
        ExcavatorGoalValidationError,
        match="outside configured range",
    ):
        validate_joint_targets(
            names,
            points,
            JOINT_LIMITS_RAD,
        )


def test_rejects_arm_above_maximum():
    names = ["arm_joint"]

    points = [
        make_point(
            math.radians(120.0),
        )
    ]

    with pytest.raises(
        ExcavatorGoalValidationError,
        match="outside configured range",
    ):
        validate_joint_targets(
            names,
            points,
            JOINT_LIMITS_RAD,
        )


def test_rejects_bucket_below_minimum():
    names = ["bucket_joint"]

    points = [
        make_point(
            math.radians(-10.0),
        )
    ]

    with pytest.raises(
        ExcavatorGoalValidationError,
        match="outside configured range",
    ):
        validate_joint_targets(
            names,
            points,
            JOINT_LIMITS_RAD,
        )


def test_rejects_swing_above_maximum():
    names = ["swing_joint"]

    points = [
        make_point(
            math.radians(100.0),
        )
    ]

    with pytest.raises(
        ExcavatorGoalValidationError,
        match="outside configured range",
    ):
        validate_joint_targets(
            names,
            points,
            JOINT_LIMITS_RAD,
        )


def test_rejects_nan_target():
    names = ["boom_joint"]

    points = [
        make_point(
            float("nan"),
        )
    ]

    with pytest.raises(
        ExcavatorGoalValidationError,
        match="not finite",
    ):
        validate_joint_targets(
            names,
            points,
            JOINT_LIMITS_RAD,
        )