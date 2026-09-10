import argparse
import sys

from excavator_control.config_loader import (
    ExcavatorConfigError,
    load_excavator_config,
)
from excavator_control.trajectory_loader import (
    ExcavatorTrajectoryError,
    load_excavator_trajectory,
)


def _get_joint_limits(
    config,
    joint_name: str,
):
    calibration = getattr(
        config,
        joint_name,
    )

    return (
        float(
            calibration.min_angle_deg
        ),
        float(
            calibration.max_angle_deg
        ),
    )


def validate_trajectory_against_config(
    config,
    trajectory,
) -> None:
    for waypoint in trajectory.waypoints:
        for joint_name in trajectory.joints:
            position_deg = (
                waypoint.positions_deg[
                    joint_name
                ]
            )

            min_angle_deg, max_angle_deg = (
                _get_joint_limits(
                    config,
                    joint_name,
                )
            )

            if (
                position_deg
                < min_angle_deg
                or position_deg
                > max_angle_deg
            ):
                raise ExcavatorTrajectoryError(
                    f"Waypoint "
                    f"'{waypoint.name}' "
                    f"{joint_name} target "
                    f"{position_deg:+.2f} deg "
                    f"is outside configured "
                    f"range "
                    f"[{min_angle_deg:+.2f}, "
                    f"{max_angle_deg:+.2f}] deg"
                )


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Validate an excavator trajectory "
            "against an excavator configuration."
        )
    )

    parser.add_argument(
        "config",
        help=(
            "Path to excavator configuration "
            "YAML"
        ),
    )

    parser.add_argument(
        "trajectory",
        help=(
            "Path to excavator trajectory "
            "YAML"
        ),
    )

    return parser.parse_args()


def main():
    args = parse_args()

    try:
        config = load_excavator_config(
            args.config
        )

        trajectory = (
            load_excavator_trajectory(
                args.trajectory
            )
        )

        validate_trajectory_against_config(
            config,
            trajectory,
        )

    except (
        ExcavatorConfigError,
        ExcavatorTrajectoryError,
    ) as exc:
        print(
            "TRAJECTORY IS INVALID"
        )
        print(
            f"Reason: {exc}"
        )
        return 1

    print(
        "TRAJECTORY IS VALID"
    )

    print(
        f"Trajectory: "
        f"{trajectory.trajectory_name}"
    )

    print(
        "Joints: "
        + ", ".join(
            trajectory.joints
        )
    )

    print(
        f"Waypoints: "
        f"{len(trajectory.waypoints)}"
    )

    return 0


if __name__ == "__main__":
    sys.exit(
        main()
    )