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


PACKAGE_ROOT = Path(__file__).resolve().parents[1]

REAL_CONFIG_PATH = (
    PACKAGE_ROOT
    / "config"
    / "excavator1.yaml"
)


def write_trajectory_yaml(
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


def make_subset_trajectory(
    boom_position: float,
):
    return {
        "trajectory_name": "boom_test",
        "description": (
            "Boom-only trajectory used "
            "for validation testing"
        ),
        "joints": [
            "boom",
        ],
        "waypoints": [
            {
                "name": "test",
                "positions": {
                    "boom": boom_position,
                },
            },
        ],
    }


def test_real_excavator_config_exists():
    assert REAL_CONFIG_PATH.is_file()


def test_valid_subset_trajectory_within_joint_limits(
    tmp_path: Path,
):
    trajectory_path = (
        write_trajectory_yaml(
            tmp_path,
            make_subset_trajectory(
                -20.0
            ),
        )
    )

    config = load_excavator_config(
        REAL_CONFIG_PATH
    )

    trajectory = (
        load_excavator_trajectory(
            trajectory_path
        )
    )

    validate_trajectory_against_config(
        config,
        trajectory,
    )


def test_reject_subset_trajectory_outside_joint_limits(
    tmp_path: Path,
):
    trajectory_path = (
        write_trajectory_yaml(
            tmp_path,
            make_subset_trajectory(
                20.0
            ),
        )
    )

    config = load_excavator_config(
        REAL_CONFIG_PATH
    )

    trajectory = (
        load_excavator_trajectory(
            trajectory_path
        )
    )

    with pytest.raises(
        ExcavatorTrajectoryError,
        match="outside configured range",
    ):
        validate_trajectory_against_config(
            config,
            trajectory,
        )


@pytest.mark.parametrize("swing_target", [-180.0, 180.0])
def test_swing_command_boundaries_are_allowed(tmp_path: Path, swing_target):
    trajectory_path = write_trajectory_yaml(tmp_path, {
        "trajectory_name": "swing_limits",
        "joints": ["swing"],
        "waypoints": [{"name": "target", "swing_direction": 1, "positions": {"swing": swing_target}}],
    })
    config = load_excavator_config(REAL_CONFIG_PATH)
    trajectory = load_excavator_trajectory(trajectory_path)
    validate_trajectory_against_config(
        config, trajectory, swing_command_limits_deg=(-180.0, 180.0)
    )


@pytest.mark.parametrize("swing_target", [-180.1, 180.1])
def test_swing_goals_outside_command_range_are_rejected(
    tmp_path: Path, swing_target
):
    trajectory_path = write_trajectory_yaml(tmp_path, {
        "trajectory_name": "swing_limits",
        "joints": ["swing"],
        "waypoints": [{"name": "target", "swing_direction": -1, "positions": {"swing": swing_target}}],
    })
    config = load_excavator_config(REAL_CONFIG_PATH)
    trajectory = load_excavator_trajectory(trajectory_path)
    with pytest.raises(ExcavatorTrajectoryError, match="outside configured range"):
        validate_trajectory_against_config(
            config, trajectory, swing_command_limits_deg=(-180.0, 180.0)
        )
