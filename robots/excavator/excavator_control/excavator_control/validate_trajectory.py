from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Dict

from excavator_control.config_loader import (
    ExcavatorConfig,
    ExcavatorConfigError,
    load_excavator_config,
)

from excavator_control.trajectory_loader import (
    ExcavatorTrajectory,
    ExcavatorTrajectoryError,
    load_excavator_trajectory,
)


def get_joint_limits(
    config: ExcavatorConfig,
) -> Dict[str, tuple[float, float]]:
    return {
        "boom": (
            config.boom.min_angle_deg,
            config.boom.max_angle_deg,
        ),

        "arm": (
            config.arm.min_angle_deg,
            config.arm.max_angle_deg,
        ),

        "bucket": (
            config.bucket.min_angle_deg,
            config.bucket.max_angle_deg,
        ),

        "swing": (
            config.swing.min_angle_deg,
            config.swing.max_angle_deg,
        ),
    }


def validate_joint_target(
    joint_name: str,
    angle_deg: float,
    min_angle_deg: float,
    max_angle_deg: float,
    waypoint_name: str,
) -> None:
    if not (
        min_angle_deg
        <= angle_deg
        <= max_angle_deg
    ):
        raise ExcavatorTrajectoryError(
            f"Waypoint '{waypoint_name}': "
            f"{joint_name} target "
            f"{angle_deg} deg is outside configured range "
            f"[{min_angle_deg}, {max_angle_deg}] deg"
        )


def validate_trajectory_against_config(
    trajectory: ExcavatorTrajectory,
    config: ExcavatorConfig,
) -> None:
    limits = get_joint_limits(
        config
    )

    for waypoint in trajectory.waypoints:

        validate_joint_target(
            "swing",
            waypoint.swing_deg,
            limits["swing"][0],
            limits["swing"][1],
            waypoint.name,
        )

        validate_joint_target(
            "boom",
            waypoint.boom_deg,
            limits["boom"][0],
            limits["boom"][1],
            waypoint.name,
        )

        validate_joint_target(
            "arm",
            waypoint.arm_deg,
            limits["arm"][0],
            limits["arm"][1],
            waypoint.name,
        )

        validate_joint_target(
            "bucket",
            waypoint.bucket_deg,
            limits["bucket"][0],
            limits["bucket"][1],
            waypoint.name,
        )


def print_summary(
    trajectory: ExcavatorTrajectory,
    trajectory_path: Path,
    config: ExcavatorConfig,
    config_path: Path,
) -> None:
    print("=" * 60)
    print("Excavator Trajectory Validation")
    print("=" * 60)

    print()

    print(
        f"Excavator      : "
        f"{config.excavator_name}"
    )

    print(
        f"Configuration  : "
        f"{config_path}"
    )

    print(
        f"Trajectory     : "
        f"{trajectory_path}"
    )

    print()

    print(
        f"Trajectory name: "
        f"{trajectory.trajectory_name}"
    )

    print(
        f"Waypoints      : "
        f"{len(trajectory.waypoints)}"
    )

    print()

    print("Joint limits:")

    for joint_name, limits in get_joint_limits(
        config
    ).items():

        print(
            f"  {joint_name:8s}: "
            f"{limits[0]:8.3f} "
            f"to "
            f"{limits[1]:8.3f} deg"
        )

    print()

    print("Waypoints:")

    for waypoint in trajectory.waypoints:

        print(
            f"  {waypoint.name}"
        )

        print(
            f"    swing : "
            f"{waypoint.swing_deg:8.3f} deg"
        )

        print(
            f"    boom  : "
            f"{waypoint.boom_deg:8.3f} deg"
        )

        print(
            f"    arm   : "
            f"{waypoint.arm_deg:8.3f} deg"
        )

        print(
            f"    bucket: "
            f"{waypoint.bucket_deg:8.3f} deg"
        )

    print()

    print(
        "RESULT: TRAJECTORY IS VALID"
    )

    print()


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Validate an excavator trajectory "
            "against a machine configuration."
        )
    )

    parser.add_argument(
        "config",
        type=Path,
        help=(
            "Excavator configuration YAML file"
        ),
    )

    parser.add_argument(
        "trajectory",
        type=Path,
        help=(
            "Excavator trajectory YAML file"
        ),
    )

    args = parser.parse_args()

    config_path = (
        args.config
        .expanduser()
        .resolve()
    )

    trajectory_path = (
        args.trajectory
        .expanduser()
        .resolve()
    )

    try:

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

        print_summary(
            trajectory,
            trajectory_path,
            config,
            config_path,
        )

    except (
        ExcavatorConfigError,
        ExcavatorTrajectoryError,
    ) as exc:

        print("=" * 60)
        print("Excavator Trajectory Validation")
        print("=" * 60)

        print()

        print(
            "RESULT: INVALID"
        )

        print()

        print(
            "Reason:"
        )

        print(
            f"  {exc}"
        )

        print()

        sys.exit(1)


if __name__ == "__main__":
    main()