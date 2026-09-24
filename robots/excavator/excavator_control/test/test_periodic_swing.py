"""Boundary and feedback tests for the physical swing motor."""

import ast
from dataclasses import dataclass
import math
from pathlib import Path
import threading
import time
from typing import Tuple

from excavator_control.swing_angles import shortest_angle_error


class FakePi:
    def set_mode(self, *args):
        pass

    def set_PWM_frequency(self, *args):
        pass

    def set_PWM_dutycycle(self, *args):
        pass

    def write(self, *args):
        pass


def make_motor():
    """Load the actual motor classes without requiring ROS on the test PC."""
    path = (Path(__file__).parents[1] / "excavator_control" /
            "excavator_trajectory_server.py")
    source = ast.parse(path.read_text(encoding="utf-8"))
    classes = [node for node in source.body if isinstance(node, ast.ClassDef)
               and node.name in ("ExternalSwingConfig", "ExternalSwingJointMotor")]
    namespace = {
        "dataclass": dataclass, "math": math, "threading": threading,
        "time": time, "Tuple": Tuple, "shortest_angle_error": shortest_angle_error,
        "clamp": lambda value, lo, hi: max(lo, min(hi, value)),
        "pigpio": type("Pigpio", (), {"OUTPUT": 1}),
    }
    exec(compile(ast.Module(body=classes, type_ignores=[]), str(path), "exec"), namespace)
    cfg = namespace["ExternalSwingConfig"](
        name="swing_joint", in1_pin=16, in2_pin=20, pwm_pin=7,
        angle_min_rad=-math.pi, angle_max_rad=math.pi, invert_motor=True,
        tolerance_rad=math.radians(15), stop_tolerance_rad=math.radians(8),
        far_error_rad=math.radians(30), far_pwm=255, near_pwm=255,
        sensor_timeout_sec=0.3, progress_timeout_sec=0.35,
        min_progress_rad=math.radians(1), pulse_err_rad=0,
        pulse_on_cycles=1, pulse_off_cycles=0,
    )
    return namespace["ExternalSwingJointMotor"](FakePi(), cfg)


def test_shortest_rotation_crosses_either_side_of_boundary():
    for current, target, expected in ((170, -145, 45), (-170, 145, -45),
                                      (179, -179, 2), (-179, 179, -2)):
        assert math.isclose(math.degrees(shortest_angle_error(
            math.radians(target), math.radians(current))), expected, abs_tol=1e-9)


def test_motor_drives_across_boundary_and_velocity_stays_small():
    motor = make_motor()
    motor.update_position(math.radians(170))
    _, err, reached, direction, pwm = motor.plan_toward_target(math.radians(-145), +1)
    assert (round(math.degrees(err)), reached, direction, pwm) == (45, False, 1, 255)
    motor.update_position(math.radians(179))
    motor.update_position(math.radians(-179))
    assert math.isclose(
        math.degrees(shortest_angle_error(motor.read_position_rad(),
                                          math.radians(179))), 2.0)
    _, err, _, direction, _ = motor.plan_toward_target(math.radians(-145), +1)
    assert round(math.degrees(err)) == 34
    assert direction == 1


def test_latched_arrival_still_rejects_stale_feedback():
    motor = make_motor()
    motor.update_position(math.radians(40))
    assert motor.plan_toward_target(math.radians(45), +1)[2]
    motor._last_sensor_receive -= 1.0
    _, _, reached, direction, pwm = motor.plan_toward_target(math.radians(45), +1)
    assert not reached and (direction, pwm) == (0, 0)
    assert "stale" in motor._swing_fault


def test_boundary_is_not_a_motor_stop():
    motor = make_motor()
    motor.update_position(math.pi)
    _, err, reached, direction, pwm = motor.plan_toward_target(
        math.radians(-135), +1)
    assert round(math.degrees(err)) == 45
    assert not reached and direction == 1 and pwm == 255


def test_watchdog_counts_forward_progress_across_wrap():
    motor = make_motor()
    motor.update_position(math.radians(179))
    motor.plan_toward_target(math.radians(-135), +1)
    motor._watch_started -= 0.4
    motor.update_position(math.radians(-179))
    _, _, reached, direction, _ = motor.plan_toward_target(math.radians(-135), +1)
    assert not reached and direction == 1
    assert motor._swing_fault is None


def test_progress_watchdog_allows_startup_then_stops_stalled_swing():
    motor = make_motor()
    motor.cfg.progress_timeout_sec = 1.5
    motor.cfg.sensor_timeout_sec = 1.0
    motor.update_position(math.radians(-90))
    motor.plan_toward_target(math.radians(-45), +1)
    motor._watch_started -= 0.4
    motor.update_position(math.radians(-90))
    assert motor.plan_toward_target(math.radians(-45), +1)[3] == 1
    assert motor._swing_fault is None
    motor._watch_started -= 1.2
    motor.update_position(math.radians(-90))
    assert motor.plan_toward_target(math.radians(-45), +1)[3:] == (0, 0)
    assert "not making sufficient progress" in motor._swing_fault


def test_explicit_negative_direction_persists_across_wrap_and_arrives():
    motor = make_motor()
    motor.update_position(math.radians(-175))
    _, err, _, direction, _ = motor.plan_toward_target(math.radians(95), -1)
    assert round(math.degrees(err)) == -90 and direction == -1
    motor.update_position(math.radians(-179))
    motor.update_position(math.radians(179))
    _, err, _, direction, _ = motor.plan_toward_target(math.radians(95), -1)
    assert round(math.degrees(err)) == -84 and direction == -1
    motor.update_position(math.radians(95))
    _, err, reached, direction, pwm = motor.plan_toward_target(math.radians(95), -1)
    assert reached and direction == 0 and pwm == 0 and abs(err) < 1e-8


def test_explicit_positive_direction_takes_long_route():
    motor = make_motor()
    motor.update_position(math.radians(-175))
    _, err, _, direction, _ = motor.plan_toward_target(math.radians(95), +1)
    assert round(math.degrees(err)) == 270 and direction == 1
    for heading in (-100, -20, 60):
        motor.update_position(math.radians(heading))
    _, err, reached, direction, _ = motor.plan_toward_target(math.radians(95), +1)
    assert not reached and round(math.degrees(err)) == 35 and direction == 1


def test_startup_corrects_only_three_joints():
    path = (Path(__file__).parents[1] / "excavator_control" /
            "excavator_trajectory_server.py")
    source = ast.parse(path.read_text(encoding="utf-8"))
    server = next(node for node in source.body if isinstance(node, ast.ClassDef)
                  and node.name == "PiExcavatorTrajectoryServer")
    method = next(node for node in server.body if isinstance(node, ast.FunctionDef)
                  and node.name == "move_to_initial_position")
    namespace = {}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), "exec"), namespace)

    class Logger:
        def warn(self, *args):
            pass

        def info(self, *args):
            pass

    class Startup:
        def __init__(self):
            self.corrected = []

        def _stop_all(self):
            pass

        def get_logger(self):
            return Logger()

        def preflight_initial_position(self):
            return True

        def _correct_initial_joint(self, name):
            self.corrected.append(name)
            return True

        def _verify_initial_position(self):
            return True

    startup = Startup()
    assert namespace["move_to_initial_position"](startup)
    assert startup.corrected == ["boom_joint", "arm_joint", "bucket_joint"]
