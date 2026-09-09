from __future__ import annotations

import argparse
import sys
from pathlib import Path

from excavator_control.config_loader import (
    ExcavatorConfigError,
    load_excavator_config,
)


def print_header(title: str) -> None:
    line = "=" * 60
    print(line)
    print(title)
    print(line)


def print_linear_joint(name, calibration) -> None:
    print(f"{name.upper()}")
    print(f"  ADC channel        : {calibration.adc_channel}")
    print(f"  Minimum angle      : {calibration.min_angle_deg:.3f} deg")
    print(f"  Maximum angle      : {calibration.max_angle_deg:.3f} deg")
    print(f"  Raw at min angle   : {calibration.raw_at_min_angle}")
    print(f"  Raw at max angle   : {calibration.raw_at_max_angle}")

    if calibration.raw_at_max_angle > calibration.raw_at_min_angle:
        direction = "raw value increases with joint angle"
    else:
        direction = "raw value decreases with joint angle"

    print(f"  Sensor direction   : {direction}")
    print()


def print_swing(calibration) -> None:
    print("SWING")
    print(f"  ADC channel        : {calibration.adc_channel}")
    print(f"  Minimum angle      : {calibration.min_angle_deg:.3f} deg")
    print(f"  Maximum angle      : {calibration.max_angle_deg:.3f} deg")
    print(f"  Calibration points : {len(calibration.angle_deg_table)}")

    if calibration.raw_table[-1] > calibration.raw_table[0]:
        direction = "raw value increases with swing angle"
    else:
        direction = "raw value decreases with swing angle"

    print(f"  Sensor direction   : {direction}")
    print()

    print("  Calibration table:")
    print()
    print("      Angle [deg]        Raw")
    print("      -----------        ---")

    for angle, raw in zip(
        calibration.angle_deg_table,
        calibration.raw_table,
    ):
        print(f"      {angle:11.3f}        {raw}")

    print()


def validate_config(config_path: Path) -> int:
    print_header("Excavator Configuration Validation")

    print(f"Configuration file:")
    print(f"  {config_path}")
    print()

    try:
        config = load_excavator_config(config_path)

    except ExcavatorConfigError as exc:
        print("RESULT: INVALID CONFIGURATION")
        print()
        print(f"Reason:")
        print(f"  {exc}")
        print()
        return 1

    except Exception as exc:
        print("RESULT: UNEXPECTED ERROR")
        print()
        print(f"{type(exc).__name__}: {exc}")
        print()
        return 2

    print("RESULT: CONFIGURATION IS VALID")
    print()

    print(f"Excavator name:")
    print(f"  {config.excavator_name}")
    print()

    print_header("Joint Calibration")

    print_linear_joint(
        "boom",
        config.boom,
    )

    print_linear_joint(
        "arm",
        config.arm,
    )

    print_linear_joint(
        "bucket",
        config.bucket,
    )

    print_swing(config.swing)

    print_header("GPIO Configuration")

    if config.gpio:
        for joint_name, joint_gpio in config.gpio.items():
            print(f"{joint_name}: {joint_gpio}")
    else:
        print("No GPIO configuration provided.")

    print()

    print_header("Control Parameters")

    if config.control:
        for key, value in config.control.items():
            print(f"{key}: {value}")
    else:
        print("No control parameters provided.")

    print()

    print_header("Validation Complete")

    print(
        "The YAML structure and calibration relationships are valid."
    )
    print(
        "This does NOT confirm that the values are correct for the "
        "physical excavator."
    )

    return 0


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Validate a CIC model excavator configuration YAML file."
        )
    )

    parser.add_argument(
        "config",
        type=Path,
        help="Path to excavator configuration YAML file",
    )

    args = parser.parse_args()

    return_code = validate_config(
        args.config.expanduser().resolve()
    )

    sys.exit(return_code)


if __name__ == "__main__":
    main()
