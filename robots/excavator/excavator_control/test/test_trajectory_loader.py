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
    filename: str = "trajectory_test.yaml",
) -> Path:
    path = tmp_path / filename

    with path.open("w", encoding="utf-8") as file:
        yaml.safe_dump(
            data,
            file,
            sort_keys=False,
        )

    return path


def make_valid_trajectory() -> dict:
    return {
        "trajectory_name": "test_cycle",

        "description": "Test excavator trajectory",

        "waypoints": [
            {
                "name": "home",

                "positions": {
                    "swing": 0.0,
                    "boom": -10.0,
                    "arm": 70.0,
                    "bucket": 20.0,
                },
            },

            {
                "name": "dig",

                "positions": {
                    "swing": 0.0,
                    "boom": -40.0,
                    "arm": 100.0,
                    "bucket": 70.0,
                },
            },
        ],
    }


def test_load_valid_trajectory(
    tmp_path: Path,
) -> None:
    path = write_yaml(
        tmp_path,
        make_valid_trajectory(),
    )

    trajectory = load_excavator_trajectory(path)

    assert trajectory.trajectory_name == "test_cycle"
    assert len(trajectory.waypoints) == 2

    assert trajectory.waypoints[0].name == "home"
    assert trajectory.waypoints[0].swing_deg == 0.0
    assert trajectory.waypoints[0].boom_deg == -10.0
    assert trajectory.waypoints[0].arm_deg == 70.0
    assert trajectory.waypoints[0].bucket_deg == 20.0


def test_reject_missing_joint(
    tmp_path: Path,
) -> None:
    data = make_valid_trajectory()

    del data["waypoints"][0]["positions"]["bucket"]

    path = write_yaml(
        tmp_path,
        data,
    )

    with pytest.raises(
        ExcavatorTrajectoryError,
        match="missing joint",
    ):
        load_excavator_trajectory(path)


def test_reject_unknown_joint(
    tmp_path: Path,
) -> None:
    data = make_valid_trajectory()

    data["waypoints"][0]["positions"]["mystery_joint"] = 123.0

    path = write_yaml(
        tmp_path,
        data,
    )

    with pytest.raises(
        ExcavatorTrajectoryError,
        match="unknown joint",
    ):
        load_excavator_trajectory(path)


def test_reject_duplicate_waypoint_name(
    tmp_path: Path,
) -> None:
    data = make_valid_trajectory()

    data["waypoints"][1]["name"] = "home"

    path = write_yaml(
        tmp_path,
        data,
    )

    with pytest.raises(
        ExcavatorTrajectoryError,
        match="Duplicate waypoint name",
    ):
        load_excavator_trajectory(path)


def test_reject_empty_waypoint_list(
    tmp_path: Path,
) -> None:
    data = make_valid_trajectory()

    data["waypoints"] = []

    path = write_yaml(
        tmp_path,
        data,
    )

    with pytest.raises(
        ExcavatorTrajectoryError,
        match="at least one waypoint",
    ):
        load_excavator_trajectory(path)
