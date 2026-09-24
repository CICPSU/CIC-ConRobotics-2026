from pathlib import Path

import pytest
import yaml

from excavator_control.trajectory_loader import (
    ExcavatorTrajectoryError,
    load_excavator_trajectory,
)


def write_yaml(
    tmp_path: Path,
    data: dict,
) -> Path:
    path = tmp_path / "trajectory.yaml"

    path.write_text(
        yaml.safe_dump(
            data,
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    return path


def make_full_trajectory():
    return {
        "trajectory_name": (
            "full_test"
        ),
        "description": (
            "Four-joint trajectory"
        ),
        "joints": [
            "swing",
            "boom",
            "arm",
            "bucket",
        ],
        "waypoints": [
            {
                "name": "start",
                "swing_direction": 1,
                "positions": {
                    "swing": 0.0,
                    "boom": -20.0,
                    "arm": 90.0,
                    "bucket": 20.0,
                },
            },
            {
                "name": "finish",
                "swing_direction": 1,
                "positions": {
                    "swing": 30.0,
                    "boom": -30.0,
                    "arm": 100.0,
                    "bucket": 40.0,
                },
            },
        ],
    }


def test_swing_direction_is_required_and_rejects_invalid_sign(tmp_path: Path):
    data = make_full_trajectory()
    data["waypoints"][0].pop("swing_direction")
    with pytest.raises(ExcavatorTrajectoryError, match="swing_direction"):
        load_excavator_trajectory(write_yaml(tmp_path, data))
    data["waypoints"][0]["swing_direction"] = 0
    with pytest.raises(ExcavatorTrajectoryError, match="swing_direction"):
        load_excavator_trajectory(write_yaml(tmp_path, data))


def test_load_valid_full_trajectory(
    tmp_path: Path,
):
    path = write_yaml(
        tmp_path,
        make_full_trajectory(),
    )

    trajectory = (
        load_excavator_trajectory(
            path
        )
    )

    assert (
        trajectory.trajectory_name
        == "full_test"
    )

    assert trajectory.joints == [
        "swing",
        "boom",
        "arm",
        "bucket",
    ]

    assert (
        len(trajectory.waypoints)
        == 2
    )

    assert (
        trajectory.waypoints[
            0
        ].positions_deg["boom"]
        == -20.0
    )


def test_load_valid_subset_trajectory(
    tmp_path: Path,
):
    data = {
        "trajectory_name": (
            "boom_only"
        ),
        "description": (
            "Boom-only test"
        ),
        "joints": [
            "boom",
        ],
        "waypoints": [
            {
                "name": "start",
                "positions": {
                    "boom": -20.0,
                },
            },
            {
                "name": "move",
                "positions": {
                    "boom": -22.0,
                },
            },
        ],
    }

    path = write_yaml(
        tmp_path,
        data,
    )

    trajectory = (
        load_excavator_trajectory(
            path
        )
    )

    assert trajectory.joints == [
        "boom"
    ]

    assert (
        trajectory.waypoints[
            1
        ].positions_deg
        == {
            "boom": -22.0
        }
    )


def test_reject_empty_joint_list(
    tmp_path: Path,
):
    data = make_full_trajectory()
    data["joints"] = []

    path = write_yaml(
        tmp_path,
        data,
    )

    with pytest.raises(
        ExcavatorTrajectoryError
    ):
        load_excavator_trajectory(
            path
        )


def test_reject_unknown_joint(
    tmp_path: Path,
):
    data = {
        "trajectory_name": "bad",
        "joints": [
            "boom",
            "banana",
        ],
        "waypoints": [
            {
                "name": "test",
                "positions": {
                    "boom": -20.0,
                    "banana": 0.0,
                },
            }
        ],
    }

    path = write_yaml(
        tmp_path,
        data,
    )

    with pytest.raises(
        ExcavatorTrajectoryError,
        match="Unknown joint",
    ):
        load_excavator_trajectory(
            path
        )


def test_reject_duplicate_joint(
    tmp_path: Path,
):
    data = {
        "trajectory_name": "bad",
        "joints": [
            "boom",
            "boom",
        ],
        "waypoints": [
            {
                "name": "test",
                "positions": {
                    "boom": -20.0,
                },
            }
        ],
    }

    path = write_yaml(
        tmp_path,
        data,
    )

    with pytest.raises(
        ExcavatorTrajectoryError,
        match="Duplicate joint",
    ):
        load_excavator_trajectory(
            path
        )


def test_reject_missing_position(
    tmp_path: Path,
):
    data = {
        "trajectory_name": "bad",
        "joints": [
            "boom",
            "arm",
        ],
        "waypoints": [
            {
                "name": "test",
                "positions": {
                    "boom": -20.0,
                },
            }
        ],
    }

    path = write_yaml(
        tmp_path,
        data,
    )

    with pytest.raises(
        ExcavatorTrajectoryError,
        match="missing positions",
    ):
        load_excavator_trajectory(
            path
        )


def test_reject_extra_position(
    tmp_path: Path,
):
    data = {
        "trajectory_name": "bad",
        "joints": [
            "boom",
        ],
        "waypoints": [
            {
                "name": "test",
                "positions": {
                    "boom": -20.0,
                    "arm": 90.0,
                },
            }
        ],
    }

    path = write_yaml(
        tmp_path,
        data,
    )

    with pytest.raises(
        ExcavatorTrajectoryError,
        match="not listed",
    ):
        load_excavator_trajectory(
            path
        )


def test_reject_duplicate_waypoint_name(
    tmp_path: Path,
):
    data = {
        "trajectory_name": "bad",
        "joints": [
            "boom",
        ],
        "waypoints": [
            {
                "name": "same",
                "positions": {
                    "boom": -20.0,
                },
            },
            {
                "name": "same",
                "positions": {
                    "boom": -25.0,
                },
            },
        ],
    }

    path = write_yaml(
        tmp_path,
        data,
    )

    with pytest.raises(
        ExcavatorTrajectoryError,
        match="Duplicate waypoint name",
    ):
        load_excavator_trajectory(
            path
        )


def test_reject_empty_waypoint_list(
    tmp_path: Path,
):
    data = {
        "trajectory_name": "bad",
        "joints": [
            "boom",
        ],
        "waypoints": [],
    }

    path = write_yaml(
        tmp_path,
        data,
    )

    with pytest.raises(
        ExcavatorTrajectoryError,
        match="at least one waypoint",
    ):
        load_excavator_trajectory(
            path
        )
