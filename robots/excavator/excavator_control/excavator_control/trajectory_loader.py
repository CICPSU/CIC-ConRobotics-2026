from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List

import yaml


class ExcavatorTrajectoryError(RuntimeError):
    """Raised when an excavator trajectory YAML file is invalid."""


@dataclass(frozen=True)
class ExcavatorWaypoint:
    name: str
    swing_deg: float
    boom_deg: float
    arm_deg: float
    bucket_deg: float


@dataclass(frozen=True)
class ExcavatorTrajectory:
    trajectory_name: str
    description: str
    waypoints: List[ExcavatorWaypoint]


JOINT_NAMES = (
    "swing",
    "boom",
    "arm",
    "bucket",
)


def _require(
    mapping: Dict[str, Any],
    key: str,
    context: str,
) -> Any:
    if key not in mapping:
        raise ExcavatorTrajectoryError(
            f"Missing required field '{key}' in {context}"
        )

    return mapping[key]


def _as_float(
    value: Any,
    field_name: str,
) -> float:
    try:
        return float(value)

    except (TypeError, ValueError) as exc:
        raise ExcavatorTrajectoryError(
            f"Field '{field_name}' must be numeric. "
            f"Got: {value!r}"
        ) from exc


def _load_waypoint(
    data: Dict[str, Any],
    index: int,
) -> ExcavatorWaypoint:
    context = f"waypoints[{index}]"

    name = _require(
        data,
        "name",
        context,
    )

    if not isinstance(name, str) or not name.strip():
        raise ExcavatorTrajectoryError(
            f"{context}.name must be a non-empty string"
        )

    positions = _require(
        data,
        "positions",
        context,
    )

    if not isinstance(positions, dict):
        raise ExcavatorTrajectoryError(
            f"{context}.positions must be a mapping"
        )

    missing_joints = [
        joint
        for joint in JOINT_NAMES
        if joint not in positions
    ]

    if missing_joints:
        raise ExcavatorTrajectoryError(
            f"{context} '{name}' is missing joint(s): "
            + ", ".join(missing_joints)
        )

    unknown_joints = [
        joint
        for joint in positions
        if joint not in JOINT_NAMES
    ]

    if unknown_joints:
        raise ExcavatorTrajectoryError(
            f"{context} '{name}' contains unknown joint(s): "
            + ", ".join(unknown_joints)
        )

    return ExcavatorWaypoint(
        name=name,
        swing_deg=_as_float(
            positions["swing"],
            f"{context}.positions.swing",
        ),
        boom_deg=_as_float(
            positions["boom"],
            f"{context}.positions.boom",
        ),
        arm_deg=_as_float(
            positions["arm"],
            f"{context}.positions.arm",
        ),
        bucket_deg=_as_float(
            positions["bucket"],
            f"{context}.positions.bucket",
        ),
    )


def load_excavator_trajectory(
    path: str | Path,
) -> ExcavatorTrajectory:
    trajectory_path = Path(path).expanduser().resolve()

    if not trajectory_path.exists():
        raise ExcavatorTrajectoryError(
            f"Trajectory file does not exist: "
            f"{trajectory_path}"
        )

    if not trajectory_path.is_file():
        raise ExcavatorTrajectoryError(
            f"Trajectory path is not a file: "
            f"{trajectory_path}"
        )

    try:
        with trajectory_path.open(
            "r",
            encoding="utf-8",
        ) as file:
            raw_data = yaml.safe_load(file)

    except yaml.YAMLError as exc:
        raise ExcavatorTrajectoryError(
            f"Failed to parse trajectory YAML: "
            f"{trajectory_path}"
        ) from exc

    if not isinstance(raw_data, dict):
        raise ExcavatorTrajectoryError(
            "Top level of trajectory YAML must be a mapping"
        )

    trajectory_name = _require(
        raw_data,
        "trajectory_name",
        "root",
    )

    if (
        not isinstance(trajectory_name, str)
        or not trajectory_name.strip()
    ):
        raise ExcavatorTrajectoryError(
            "trajectory_name must be a non-empty string"
        )

    description = raw_data.get(
        "description",
        "",
    )

    if description is None:
        description = ""

    if not isinstance(description, str):
        raise ExcavatorTrajectoryError(
            "description must be a string"
        )

    raw_waypoints = _require(
        raw_data,
        "waypoints",
        "root",
    )

    if not isinstance(raw_waypoints, list):
        raise ExcavatorTrajectoryError(
            "waypoints must be a list"
        )

    if not raw_waypoints:
        raise ExcavatorTrajectoryError(
            "Trajectory must contain at least one waypoint"
        )

    waypoints: List[ExcavatorWaypoint] = []

    waypoint_names = set()

    for index, raw_waypoint in enumerate(raw_waypoints):
        if not isinstance(raw_waypoint, dict):
            raise ExcavatorTrajectoryError(
                f"waypoints[{index}] must be a mapping"
            )

        waypoint = _load_waypoint(
            raw_waypoint,
            index,
        )

        if waypoint.name in waypoint_names:
            raise ExcavatorTrajectoryError(
                f"Duplicate waypoint name: {waypoint.name}"
            )

        waypoint_names.add(
            waypoint.name
        )

        waypoints.append(
            waypoint
        )

    return ExcavatorTrajectory(
        trajectory_name=trajectory_name,
        description=description,
        waypoints=waypoints,
    )
