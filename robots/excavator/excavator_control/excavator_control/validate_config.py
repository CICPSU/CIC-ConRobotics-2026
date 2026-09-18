from __future__ import annotations

import argparse
from pathlib import Path

from excavator_control.config_loader import load_excavator_config


def print_linear_joint(calibration) -> None:
    direction = (
        "raw value increases with joint angle"
        if calibration.raw_at_max_angle > calibration.raw_at_min_angle
        else "raw value decreases with joint angle"
    )
    print(calibration.name.upper())
    print(f"  angle range : {calibration.min_angle_deg:.3f} .. "
          f"{calibration.max_angle_deg:.3f} deg")
    print(f"  ADC channel : {calibration.adc_channel}")
    print(f"  raw endpoints: {calibration.raw_at_min_angle:.1f} .. "
          f"{calibration.raw_at_max_angle:.1f}")
    print(f"  direction   : {direction}")


def print_swing(swing) -> None:
    print("SWING")
    print(f"  angle range : {swing.min_angle_deg:.3f} .. "
          f"{swing.max_angle_deg:.3f} deg")
    print("  sensing     : external sensor adapter (JointState topic)")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Validate and summarize an excavator YAML configuration"
    )
    parser.add_argument("config", type=Path)
    args = parser.parse_args()

    config = load_excavator_config(args.config)
    print(f"EXCAVATOR: {config.excavator_name}")
    print_swing(config.swing)
    print_linear_joint(config.boom)
    print_linear_joint(config.arm)
    print_linear_joint(config.bucket)


if __name__ == "__main__":
    main()