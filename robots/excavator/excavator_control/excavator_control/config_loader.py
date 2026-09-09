from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List

import yaml


class ExcavatorConfigError(RuntimeError):
    """Raised when an excavator configuration file is invalid."""


@dataclass(frozen=True)
class LinearJointCalibration:
    name: str
    adc_channel: int
    min_angle_deg: float
    max_angle_deg: float
    raw_at_min_angle: float
    raw_at_max_angle: float


@dataclass(frozen=True)
class SwingCalibration:
    name: str
    adc_channel: int
    min_angle_deg: float
    max_angle_deg: float
    angle_deg_table: List[float]
    raw_table: List[float]


@dataclass(frozen=True)
class ExcavatorConfig:
    excavator_name: str

    boom: LinearJointCalibration
    arm: LinearJointCalibration
    bucket: LinearJointCalibration
    swing: SwingCalibration

    gpio: Dict[str, Any]
    control: Dict[str, Any]


def _require(mapping: Dict[str, Any], key: str, context: str) -> Any:
    if key not in mapping:
        raise ExcavatorConfigError(
            f"Missing required field '{key}' in {context}"
        )
    return mapping[key]


def _as_float(value: Any, field_name: str) -> float:
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise ExcavatorConfigError(
            f"Field '{field_name}' must be numeric. Got: {value!r}"
        ) from exc


def _as_int(value: Any, field_name: str) -> int:
    try:
        return int(value)
    except (TypeError, ValueError) as exc:
        raise ExcavatorConfigError(
            f"Field '{field_name}' must be an integer. Got: {value!r}"
        ) from exc


def _validate_angle_range(
    min_angle_deg: float,
    max_angle_deg: float,
    joint_name: str,
) -> None:
    if min_angle_deg >= max_angle_deg:
        raise ExcavatorConfigError(
            f"{joint_name}: min_angle_deg must be smaller than "
            f"max_angle_deg. Got {min_angle_deg} >= {max_angle_deg}"
        )


def _load_linear_joint(
    name: str,
    data: Dict[str, Any],
) -> LinearJointCalibration:
    adc_channel = _as_int(
        _require(data, "adc_channel", name),
        f"{name}.adc_channel",
    )

    min_angle_deg = _as_float(
        _require(data, "min_angle_deg", name),
        f"{name}.min_angle_deg",
    )

    max_angle_deg = _as_float(
        _require(data, "max_angle_deg", name),
        f"{name}.max_angle_deg",
    )

    raw_at_min_angle = _as_float(
        _require(data, "raw_at_min_angle", name),
        f"{name}.raw_at_min_angle",
    )

    raw_at_max_angle = _as_float(
        _require(data, "raw_at_max_angle", name),
        f"{name}.raw_at_max_angle",
    )

    _validate_angle_range(
        min_angle_deg,
        max_angle_deg,
        name,
    )

    if raw_at_min_angle == raw_at_max_angle:
        raise ExcavatorConfigError(
            f"{name}: raw_at_min_angle and raw_at_max_angle "
            "cannot be identical"
        )

    return LinearJointCalibration(
        name=name,
        adc_channel=adc_channel,
        min_angle_deg=min_angle_deg,
        max_angle_deg=max_angle_deg,
        raw_at_min_angle=raw_at_min_angle,
        raw_at_max_angle=raw_at_max_angle,
    )


def _load_swing(
    name: str,
    data: Dict[str, Any],
) -> SwingCalibration:
    adc_channel = _as_int(
        _require(data, "adc_channel", name),
        f"{name}.adc_channel",
    )

    min_angle_deg = _as_float(
        _require(data, "min_angle_deg", name),
        f"{name}.min_angle_deg",
    )

    max_angle_deg = _as_float(
        _require(data, "max_angle_deg", name),
        f"{name}.max_angle_deg",
    )

    _validate_angle_range(
        min_angle_deg,
        max_angle_deg,
        name,
    )

    angle_deg_table_raw = _require(
        data,
        "angle_deg_table",
        name,
    )

    raw_table_raw = _require(
        data,
        "raw_table",
        name,
    )

    if not isinstance(angle_deg_table_raw, list):
        raise ExcavatorConfigError(
            f"{name}.angle_deg_table must be a list"
        )

    if not isinstance(raw_table_raw, list):
        raise ExcavatorConfigError(
            f"{name}.raw_table must be a list"
        )

    angle_deg_table = [
        _as_float(v, f"{name}.angle_deg_table")
        for v in angle_deg_table_raw
    ]

    raw_table = [
        _as_float(v, f"{name}.raw_table")
        for v in raw_table_raw
    ]

    if len(angle_deg_table) != len(raw_table):
        raise ExcavatorConfigError(
            f"{name}: angle_deg_table and raw_table must "
            f"have the same length. Got "
            f"{len(angle_deg_table)} and {len(raw_table)}"
        )

    if len(angle_deg_table) < 2:
        raise ExcavatorConfigError(
            f"{name}: at least two calibration points are required"
        )

    for index in range(len(angle_deg_table) - 1):
        if angle_deg_table[index] >= angle_deg_table[index + 1]:
            raise ExcavatorConfigError(
                f"{name}.angle_deg_table must be strictly increasing"
            )

    raw_increasing = all(
        raw_table[i] < raw_table[i + 1]
        for i in range(len(raw_table) - 1)
    )

    raw_decreasing = all(
        raw_table[i] > raw_table[i + 1]
        for i in range(len(raw_table) - 1)
    )

    if not (raw_increasing or raw_decreasing):
        raise ExcavatorConfigError(
            f"{name}.raw_table must be strictly monotonic "
            "(either increasing or decreasing)"
        )

    if angle_deg_table[0] < min_angle_deg:
        raise ExcavatorConfigError(
            f"{name}: first calibration angle "
            f"{angle_deg_table[0]} is below min_angle_deg "
            f"{min_angle_deg}"
        )

    if angle_deg_table[-1] > max_angle_deg:
        raise ExcavatorConfigError(
            f"{name}: last calibration angle "
            f"{angle_deg_table[-1]} is above max_angle_deg "
            f"{max_angle_deg}"
        )

    return SwingCalibration(
        name=name,
        adc_channel=adc_channel,
        min_angle_deg=min_angle_deg,
        max_angle_deg=max_angle_deg,
        angle_deg_table=angle_deg_table,
        raw_table=raw_table,
    )


def load_excavator_config(path: str | Path) -> ExcavatorConfig:
    config_path = Path(path).expanduser().resolve()

    if not config_path.exists():
        raise ExcavatorConfigError(
            f"Excavator config file does not exist: {config_path}"
        )

    if not config_path.is_file():
        raise ExcavatorConfigError(
            f"Excavator config path is not a file: {config_path}"
        )

    try:
        with config_path.open(
            "r",
            encoding="utf-8",
        ) as file:
            raw_config = yaml.safe_load(file)
    except yaml.YAMLError as exc:
        raise ExcavatorConfigError(
            f"Failed to parse YAML file: {config_path}"
        ) from exc

    if not isinstance(raw_config, dict):
        raise ExcavatorConfigError(
            "Top level of excavator configuration must be a mapping"
        )

    excavator_name = str(
        _require(
            raw_config,
            "excavator_name",
            "root",
        )
    )

    joints = _require(
        raw_config,
        "joints",
        "root",
    )

    if not isinstance(joints, dict):
        raise ExcavatorConfigError(
            "'joints' must be a mapping"
        )

    boom_data = _require(
        joints,
        "boom",
        "joints",
    )

    arm_data = _require(
        joints,
        "arm",
        "joints",
    )

    bucket_data = _require(
        joints,
        "bucket",
        "joints",
    )

    swing_data = _require(
        joints,
        "swing",
        "joints",
    )

    for name, data in [
        ("boom", boom_data),
        ("arm", arm_data),
        ("bucket", bucket_data),
        ("swing", swing_data),
    ]:
        if not isinstance(data, dict):
            raise ExcavatorConfigError(
                f"joints.{name} must be a mapping"
            )

    gpio = raw_config.get("gpio", {})
    control = raw_config.get("control", {})

    if not isinstance(gpio, dict):
        raise ExcavatorConfigError(
            "'gpio' must be a mapping"
        )

    if not isinstance(control, dict):
        raise ExcavatorConfigError(
            "'control' must be a mapping"
        )

    return ExcavatorConfig(
        excavator_name=excavator_name,
        boom=_load_linear_joint(
            "boom",
            boom_data,
        ),
        arm=_load_linear_joint(
            "arm",
            arm_data,
        ),
        bucket=_load_linear_joint(
            "bucket",
            bucket_data,
        ),
        swing=_load_swing(
            "swing",
            swing_data,
        ),
        gpio=gpio,
        control=control,
    )


def linear_raw_to_angle(
    raw_value: float,
    calibration: LinearJointCalibration,
) -> float:
    raw_span = (
        calibration.raw_at_max_angle
        - calibration.raw_at_min_angle
    )

    angle_span = (
        calibration.max_angle_deg
        - calibration.min_angle_deg
    )

    ratio = (
        raw_value
        - calibration.raw_at_min_angle
    ) / raw_span

    angle_deg = (
        calibration.min_angle_deg
        + ratio * angle_span
    )

    return angle_deg


def linear_angle_to_raw(
    angle_deg: float,
    calibration: LinearJointCalibration,
) -> float:
    angle_span = (
        calibration.max_angle_deg
        - calibration.min_angle_deg
    )

    raw_span = (
        calibration.raw_at_max_angle
        - calibration.raw_at_min_angle
    )

    ratio = (
        angle_deg
        - calibration.min_angle_deg
    ) / angle_span

    raw_value = (
        calibration.raw_at_min_angle
        + ratio * raw_span
    )

    return raw_value


def swing_raw_to_angle(
    raw_value: float,
    calibration: SwingCalibration,
) -> float:
    raw_table = calibration.raw_table
    angle_table = calibration.angle_deg_table

    increasing = raw_table[-1] > raw_table[0]

    if increasing:
        if raw_value <= raw_table[0]:
            return angle_table[0]

        if raw_value >= raw_table[-1]:
            return angle_table[-1]
    else:
        if raw_value >= raw_table[0]:
            return angle_table[0]

        if raw_value <= raw_table[-1]:
            return angle_table[-1]

    for index in range(len(raw_table) - 1):
        raw_a = raw_table[index]
        raw_b = raw_table[index + 1]

        if increasing:
            inside = raw_a <= raw_value <= raw_b
        else:
            inside = raw_a >= raw_value >= raw_b

        if not inside:
            continue

        angle_a = angle_table[index]
        angle_b = angle_table[index + 1]

        ratio = (
            raw_value - raw_a
        ) / (
            raw_b - raw_a
        )

        return (
            angle_a
            + ratio * (angle_b - angle_a)
        )

    raise ExcavatorConfigError(
        f"Could not interpolate swing raw value: {raw_value}"
    )


def clamp_angle(
    angle_deg: float,
    min_angle_deg: float,
    max_angle_deg: float,
) -> float:
    return max(
        min_angle_deg,
        min(
            max_angle_deg,
            angle_deg,
        ),
    )