from pathlib import Path

import pytest
import yaml

from excavator_control.config_loader import (
    load_excavator_config,
)

from excavator_control.trajectory_loader import (
    ExcavatorTrajectoryError,
    load_excavator_trajectory,
)

from excavator_control.validate_trajectory import (
    validate_trajectory_against_config,
)


def write_yaml(
    tmp_path: Path,
    filename: str,
    data: dict,
) -> Path:
    path = tmp_path / filename

    with path.open("w", encoding="utf-8") as file:
        yaml.safe_dump(
            data,
            file,
            sort_keys=False,
        )

    return path


def make_config() -> dict:
    return {
        "excavator_name": "excavator_test",

        "joints": {
            "boom": {
                "adc_channel": 0,
                "min_angle_deg": -60.0,
                "max_angle_deg": 5.0,
                "raw_at_min_angle": 1000,
                "raw_at_max_angle": 2000,
            },

            "arm": {
                "adc_channel": 1,
                "min_angle_deg": 52.0,
                "max_angle_deg": 112.0,
                "raw_at_min_angle": 3000,
                "raw_at_max_angle": 2000,
            },

            "bucket": {
                "adc_channel": 2,
                "min_angle_deg": 0.0,
                "max_angle_deg": 90.0,
                "raw_at_min_angle": 500,
                "raw_at_max_angle": 2500,
            },

            "swing": {
                "adc_channel": 3,
                "min_angle_deg": -90.0,
                "max_angle_deg": 90.0,
                "angle_deg_table": [
                    -90.0,
                    0.0,
                    90.0,
                ],
                "raw_table": [
                    1000,
                    2000,
                    3000,
                ],
            },
        },

        "gpio": {},

        "control": {},
    }


def make_trajectory() -> dict:
    return {
        "trajectory_name": "test",

        "waypoints": [
            {
                "name": "point1",

                "positions": {
                    "swing": 0.0,
                    "boom": -20.0,
                    "arm": 80.0,
                    "bucket": 45.0,
                },
            },
        ],
    }


def test_valid_trajectory_within_joint_limits(
    tmp_path: Path,
) -> None:
    config_path = write_yaml(
        tmp_path,
        "config.yaml",
        make_config(),
    )

    trajectory_path = write_yaml(
        tmp_path,
        "trajectory.yaml",
        make_trajectory(),
    )

    config = load_excavator_config(
        config_path
    )

    trajectory = load_excavator_trajectory(
        trajectory_path
    )

    validate_trajectory_against_config(
        trajectory,
        config,
    )


def test_reject_trajectory_outside_joint_limits(
    tmp_path: Path,
) -> None:
    config_data = make_config()

    trajectory_data = make_trajectory()

    trajectory_data["waypoints"][0]["positions"]["boom"] = -80.0

    config_path = write_yaml(
        tmp_path,
        "config.yaml",
        config_data,
    )

    trajectory_path = write_yaml(
        tmp_path,
        "trajectory.yaml",
        trajectory_data,
    )

    config = load_excavator_config(
        config_path
    )

    trajectory = load_excavator_trajectory(
        trajectory_path
    )

    with pytest.raises(
        ExcavatorTrajectoryError,
        match="outside configured range",
    ):
        validate_trajectory_against_config(
            trajectory,
            config,
        )
