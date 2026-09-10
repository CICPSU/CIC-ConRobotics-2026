from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List

import math
import yaml


class ExcavatorTrajectoryError(RuntimeError):
    """Raised when an excavator trajectory file is invalid."""


VALID_JOINTS = (
    "swing",
    "boom",
    "arm",
    "bucket",
)


@dataclass(frozen=True)
class ExcavatorWaypoint:
    name: str
    positions_deg: Dict[str, float]


@dataclass(frozen=True)
class ExcavatorTrajectory:
    trajectory_name: str
    description: str
    joints: List[str]
    waypoints: List[ExcavatorWaypoint]


def _require_mapping(value, context: str) -> dict:
    if not isinstance(value, dict):
        raise ExcavatorTrajectoryError(
            f"{context} must be a mapping"
        )
    return value


def _require_list(value, context: str) -> list:
    if not isinstance(value, list):
        raise ExcavatorTrajectoryError(
            f"{context} must be a list"
        )
    return value


def _require_nonempty_string(
    value,
    context: str,
) -> str:
    if not isinstance(value, str):
        raise ExcavatorTrajectoryError(
            f"{context} must be a string"
        )

    value = value.strip()

    if not value:
        raise ExcavatorTrajectoryError(
            f"{context} must not be empty"
        )

    return value


def _require_finite_number(
    value,
    context: str,
) -> float:
    if isinstance(value, bool):
        raise ExcavatorTrajectoryError(
            f"{context} must be numeric"
        )

    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ExcavatorTrajectoryError(
            f"{context} must be numeric"
        ) from exc

    if not math.isfinite(number):
        raise ExcavatorTrajectoryError(
            f"{context} must be finite"
        )

    return number


def _load_yaml(path: Path) -> dict:
    if not path.is_file():
        raise ExcavatorTrajectoryError(
            f"Trajectory file does not exist: {path}"
        )

    try:
        with path.open(
            "r",
            encoding="utf-8",
        ) as stream:
            data = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        raise ExcavatorTrajectoryError(
            f"Failed to parse trajectory YAML: {exc}"
        ) from exc
    except OSError as exc:
        raise ExcavatorTrajectoryError(
            f"Failed to read trajectory file: {exc}"
        ) from exc

    if data is None:
        raise ExcavatorTrajectoryError(
            "Trajectory file is empty"
        )

    return _require_mapping(
        data,
        "Trajectory root",
    )


def _parse_joints(data: dict) -> List[str]:
    if "joints" not in data:
        raise ExcavatorTrajectoryError(
            "Trajectory is missing 'joints'"
        )

    raw_joints = _require_list(
        data["joints"],
        "'joints'",
    )

    if not raw_joints:
        raise ExcavatorTrajectoryError(
            "Trajectory must contain at least one joint"
        )

    joints: List[str] = []

    for index, raw_joint in enumerate(
        raw_joints
    ):
        joint = _require_nonempty_string(
            raw_joint,
            f"joints[{index}]",
        )

        if joint not in VALID_JOINTS:
            raise ExcavatorTrajectoryError(
                f"Unknown joint '{joint}'. "
                f"Valid joints are: "
                f"{', '.join(VALID_JOINTS)}"
            )

        if joint in joints:
            raise ExcavatorTrajectoryError(
                f"Duplicate joint '{joint}'"
            )

        joints.append(joint)

    return joints


def _parse_waypoints(
    data: dict,
    joints: List[str],
) -> List[ExcavatorWaypoint]:
    if "waypoints" not in data:
        raise ExcavatorTrajectoryError(
            "Trajectory is missing 'waypoints'"
        )

    raw_waypoints = _require_list(
        data["waypoints"],
        "'waypoints'",
    )

    if not raw_waypoints:
        raise ExcavatorTrajectoryError(
            "Trajectory must contain at least one waypoint"
        )

    waypoints: List[ExcavatorWaypoint] = []
    waypoint_names = set()

    for waypoint_index, raw_waypoint in enumerate(
        raw_waypoints
    ):
        waypoint = _require_mapping(
            raw_waypoint,
            f"waypoints[{waypoint_index}]",
        )

        name = _require_nonempty_string(
            waypoint.get("name"),
            f"waypoints[{waypoint_index}].name",
        )

        if name in waypoint_names:
            raise ExcavatorTrajectoryError(
                f"Duplicate waypoint name '{name}'"
            )

        waypoint_names.add(name)

        if "positions" not in waypoint:
            raise ExcavatorTrajectoryError(
                f"Waypoint '{name}' is missing "
                "'positions'"
            )

        raw_positions = _require_mapping(
            waypoint["positions"],
            f"Waypoint '{name}' positions",
        )

        position_names = set(
            raw_positions.keys()
        )
        joint_names = set(joints)

        missing = joint_names - position_names
        extra = position_names - joint_names

        if missing:
            raise ExcavatorTrajectoryError(
                f"Waypoint '{name}' is missing "
                f"positions for joints: "
                f"{sorted(missing)}"
            )

        if extra:
            raise ExcavatorTrajectoryError(
                f"Waypoint '{name}' contains "
                f"positions for joints not listed "
                f"in trajectory 'joints': "
                f"{sorted(extra)}"
            )

        positions_deg: Dict[str, float] = {}

        for joint in joints:
            positions_deg[joint] = (
                _require_finite_number(
                    raw_positions[joint],
                    (
                        f"Waypoint '{name}' "
                        f"position '{joint}'"
                    ),
                )
            )

        waypoints.append(
            ExcavatorWaypoint(
                name=name,
                positions_deg=positions_deg,
            )
        )

    return waypoints


def load_excavator_trajectory(
    path,
) -> ExcavatorTrajectory:
    trajectory_path = Path(path).expanduser()

    data = _load_yaml(
        trajectory_path
    )

    trajectory_name = (
        _require_nonempty_string(
            data.get("trajectory_name"),
            "'trajectory_name'",
        )
    )

    description = data.get(
        "description",
        "",
    )

    if description is None:
        description = ""

    if not isinstance(description, str):
        raise ExcavatorTrajectoryError(
            "'description' must be a string"
        )

    description = description.strip()

    joints = _parse_joints(
        data
    )

    waypoints = _parse_waypoints(
        data,
        joints,
    )

    return ExcavatorTrajectory(
        trajectory_name=trajectory_name,
        description=description,
        joints=joints,
        waypoints=waypoints,
    )