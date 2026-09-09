from pathlib import Path

import pytest
import yaml

from excavator_control.config_loader import (
    ExcavatorConfigError,
    linear_angle_to_raw,
    linear_raw_to_angle,
    load_excavator_config,
    swing_raw_to_angle,
)


def write_yaml(
    tmp_path: Path,
    data: dict,
    filename: str = "excavator_test.yaml",
) -> Path:
    path = tmp_path / filename

    with path.open("w", encoding="utf-8") as file:
        yaml.safe_dump(
            data,
            file,
            sort_keys=False,
        )

    return path


def make_valid_config() -> dict:
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
                    -45.0,
                    0.0,
                    45.0,
                    90.0,
                ],
                "raw_table": [
                    1000,
                    1500,
                    2000,
                    2500,
                    3000,
                ],
            },
        },

        "gpio": {
            "boom": {
                "forward_pin": 1,
                "reverse_pin": 2,
            },
            "arm": {
                "forward_pin": 3,
                "reverse_pin": 4,
            },
            "bucket": {
                "forward_pin": 5,
                "reverse_pin": 6,
            },
            "swing": {
                "forward_pin": 7,
                "reverse_pin": 8,
            },
        },

        "control": {
            "control_rate_hz": 20.0,

            "goal_tolerance_deg": {
                "boom": 2.0,
                "arm": 2.0,
                "bucket": 2.0,
                "swing": 2.0,
            },

            "timeout_sec": {
                "boom": 20.0,
                "arm": 20.0,
                "bucket": 20.0,
                "swing": 20.0,
            },
        },
    }


def test_load_valid_config(
    tmp_path: Path,
) -> None:
    path = write_yaml(
        tmp_path,
        make_valid_config(),
    )

    config = load_excavator_config(path)

    assert config.excavator_name == "excavator_test"

    assert config.boom.min_angle_deg == -60.0
    assert config.boom.max_angle_deg == 5.0

    assert config.arm.min_angle_deg == 52.0
    assert config.arm.max_angle_deg == 112.0

    assert config.bucket.min_angle_deg == 0.0
    assert config.bucket.max_angle_deg == 90.0

    assert config.swing.min_angle_deg == -90.0
    assert config.swing.max_angle_deg == 90.0


def test_reject_identical_linear_raw_endpoints(
    tmp_path: Path,
) -> None:
    data = make_valid_config()

    data["joints"]["boom"]["raw_at_min_angle"] = 1234
    data["joints"]["boom"]["raw_at_max_angle"] = 1234

    path = write_yaml(
        tmp_path,
        data,
    )

    with pytest.raises(
        ExcavatorConfigError,
        match="cannot be identical",
    ):
        load_excavator_config(path)


def test_linear_raw_to_angle_increasing_sensor(
    tmp_path: Path,
) -> None:
    path = write_yaml(
        tmp_path,
        make_valid_config(),
    )

    config = load_excavator_config(path)

    angle = linear_raw_to_angle(
        1500,
        config.boom,
    )

    assert angle == pytest.approx(-27.5)


def test_linear_raw_to_angle_decreasing_sensor(
    tmp_path: Path,
) -> None:
    path = write_yaml(
        tmp_path,
        make_valid_config(),
    )

    config = load_excavator_config(path)

    angle = linear_raw_to_angle(
        2500,
        config.arm,
    )

    assert angle == pytest.approx(82.0)


def test_linear_angle_to_raw(
    tmp_path: Path,
) -> None:
    path = write_yaml(
        tmp_path,
        make_valid_config(),
    )

    config = load_excavator_config(path)

    raw = linear_angle_to_raw(
        45.0,
        config.bucket,
    )

    assert raw == pytest.approx(1500.0)


def test_swing_interpolation(
    tmp_path: Path,
) -> None:
    path = write_yaml(
        tmp_path,
        make_valid_config(),
    )

    config = load_excavator_config(path)

    angle = swing_raw_to_angle(
        2250,
        config.swing,
    )

    assert angle == pytest.approx(22.5)