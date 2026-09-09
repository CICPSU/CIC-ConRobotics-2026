#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#in terminal one - run sudo pigpiod then run 
#python3 excavator_trajectory_server.py --mode pi
#in terminal two run 
#python3 scoop_trajectory_client.py --spec degrees.yaml
"""
excavator_trajectory_server.py  –  UNIFIED server for Pi + ROS/Isaac Sim

Auto-detects which machine it is running on:

  Raspberry Pi  → real-robot mode
    - imports pigpio, ADS1115
    - controls GPIO motors
    - reads potentiometers via ADS1115 ADC
    - publishes real joint states to /joint_states

  ROS / Isaac Sim computer  → simulation bridge mode
    - skips pigpio and ADS1115 entirely
    - publishes sensor_msgs/JointState to /joint_command for Isaac Sim
    - trajectories are interpolated in software (no physical hardware)

Detection order (first match wins):
  1. --mode pi | --mode sim  (explicit CLI flag, most reliable)
  2. EXCAVATOR_MODE=pi | sim  (environment variable)
  3. pigpio available AND pigpiod daemon is reachable  →  Pi mode
  4. Fallback  →  sim mode

Usage:
  # On Raspberry Pi (auto-detect or explicit):
  python3 excavator_trajectory_server.py
  python3 excavator_trajectory_server.py --mode pi

  # On ROS/Isaac Sim computer:
  python3 excavator_trajectory_server.py --mode sim

Install (Pi only):
  pip3 install pigpio adafruit-circuitpython-ads1x15
  sudo pigpiod

Enable I2C on Pi:
  sudo raspi-config → Interface Options → I2C → Enable
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
import threading
from dataclasses import dataclass
from pathlib import Path

import yaml
from typing import Dict, List, Optional, Protocol, Sequence, Tuple

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from ament_index_python.packages import get_package_share_directory

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from excavator_control.config_loader import load_excavator_config
from excavator_control.goal_validation import (
    ExcavatorGoalValidationError,
    validate_joint_targets,
)


# ================================================================
# Runtime configuration
# ================================================================

def _default_config_path() -> str:
    """Return the installed excavator1.yaml path."""
    share_dir = get_package_share_directory("excavator_control")
    return str(Path(share_dir) / "config" / "excavator1.yaml")


def _load_runtime_yaml(config_path: str) -> dict:
    """Load sections not yet represented directly by ExcavatorConfig."""
    path = Path(config_path).expanduser().resolve()
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise RuntimeError(f"Excavator configuration must be a YAML mapping: {path}")
    return data


def _require_mapping(data: dict, key: str) -> dict:
    value = data.get(key)
    if not isinstance(value, dict):
        raise RuntimeError(f"Missing or invalid '{key}' mapping in excavator configuration")
    return value


def _linear_pot_endpoints(calibration):
    """
    Convert semantic min/max-angle calibration into the raw-ordered endpoint
    convention used internally by PotentiometerJointMotor.

    Returns:
        adc_low, adc_high, angle_at_adc_low_rad, angle_at_adc_high_rad
    """
    raw_at_min = float(calibration.raw_at_min_angle)
    raw_at_max = float(calibration.raw_at_max_angle)
    min_angle = math.radians(float(calibration.min_angle_deg))
    max_angle = math.radians(float(calibration.max_angle_deg))

    if raw_at_min < raw_at_max:
        return int(round(raw_at_min)), int(round(raw_at_max)), min_angle, max_angle

    return int(round(raw_at_max)), int(round(raw_at_min)), max_angle, min_angle


def _configure_swing_from_yaml(config) -> None:
    """Populate the legacy swing lookup globals from ExcavatorConfig."""
    global SWING_RAW_MIN, SWING_RAW_MAX
    global SWING_RAW_PHYS_MIN, SWING_RAW_PHYS_MAX
    global SWING_RAW_REJECT_LO, SWING_RAW_REJECT_HI
    global SWING_RAW_TRUSTED_MAX
    global SWING_RAW_TABLE, SWING_SWEEP_DEG, SWING_ANGLE_TABLE
    global SWING_ANGLE_MIN_RAD, SWING_ANGLE_MAX_RAD
    global SWING_USABLE_MAX_DEG, SWING_USABLE_MAX_RAD
    global SWING_RAW_ZERO

    raw_table = [int(round(v)) for v in config.swing.raw_table]
    angle_table = [float(v) for v in config.swing.angle_deg_table]

    if len(raw_table) < 2 or len(raw_table) != len(angle_table):
        raise RuntimeError("Swing calibration table is invalid")

    SWING_RAW_TABLE = raw_table
    SWING_SWEEP_DEG = angle_table
    SWING_ANGLE_TABLE = [math.radians(v) for v in SWING_SWEEP_DEG]

    SWING_RAW_MIN = min(SWING_RAW_TABLE)
    SWING_RAW_MAX = max(SWING_RAW_TABLE)
    SWING_RAW_PHYS_MIN = SWING_RAW_MIN
    SWING_RAW_PHYS_MAX = SWING_RAW_MAX
    SWING_RAW_REJECT_LO = SWING_RAW_PHYS_MIN - SWING_RAW_MARGIN
    SWING_RAW_REJECT_HI = SWING_RAW_PHYS_MAX + SWING_RAW_MARGIN
    SWING_RAW_TRUSTED_MAX = SWING_RAW_MAX

    SWING_ANGLE_MIN_RAD = math.radians(float(config.swing.min_angle_deg))
    SWING_ANGLE_MAX_RAD = math.radians(float(config.swing.max_angle_deg))
    SWING_USABLE_MAX_DEG = float(config.swing.max_angle_deg)
    SWING_USABLE_MAX_RAD = math.radians(SWING_USABLE_MAX_DEG)

    SWING_RAW_ZERO = swing_rad_to_raw(0.0)


# ================================================================
# Platform detection
# ================================================================

def _detect_mode(cli_mode: Optional[str]) -> str:
    """
    Returns 'pi' or 'sim'.
    Resolution order:
      1. CLI --mode argument
      2. EXCAVATOR_MODE environment variable
      3. pigpio reachable → 'pi'
      4. default → 'sim'
    """
    # 1. Explicit CLI flag
    if cli_mode is not None:
        m = cli_mode.strip().lower()
        if m not in ("pi", "sim"):
            raise ValueError(f"--mode must be 'pi' or 'sim', got: {cli_mode!r}")
        return m

    # 2. Environment variable
    env = os.environ.get("EXCAVATOR_MODE", "").strip().lower()
    if env in ("pi", "sim"):
        return env

    # 3. Try to import pigpio and connect to the daemon
    try:
        import pigpio as _pigpio  # noqa: F401
        _test = _pigpio.pi()
        if _test.connected:
            _test.stop()
            return "pi"
        _test.stop()
    except Exception:
        pass

    # 4. Default to sim
    return "sim"


# ================================================================
# Helpers (shared by both modes)
# Next set of functions are sort of like a schedule. 
# by 0 second , be here, etc 
# ================================================================

def duration_to_seconds(d) -> float:
    return float(d.sec) + float(d.nanosec) * 1e-9


def seconds_to_duration(t: float) -> Duration:
    sec = int(math.floor(t))
    nanosec = int((t - sec) * 1e9)
    return Duration(sec=sec, nanosec=nanosec)


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


# ================================================================
# SWING PHOTO-REFLECTOR CALIBRATION  (ADS1115 channel 3)
# ================================================================
# THIS IS THE ONLY SWING CALIBRATION IN THE FILE.
# The swing does NOT use PotJointConfig.adc_min / adc_center / adc_max.
#
# Measured sweep after the gradient was changed (raw ADC / voltage @
# gain=1), monotonic. Labels are SWEEP degrees along the physical travel.
#     0 sweep-deg -> 10848 / 1.356 V
#    15 sweep-deg -> 11472 / 1.434 V
#    45 sweep-deg -> 14224 / 1.778 V
#    75 sweep-deg -> 18368 / 2.296 V
#    90 sweep-deg -> 18736 / 2.342 V
#
# WARNING - THIS GRADIENT IS MUCH WEAKER THAN THE PREVIOUS ONE.
# Total span is 7888 counts over 90 deg, down from 15072. Worse, the
# sensitivity is wildly uneven:
#     0-15 deg:  41.6 counts/deg
#    15-45 deg:  91.7 counts/deg
#    45-75 deg: 138.1 counts/deg
#    75-90 deg:  24.5 counts/deg   <-- nearly saturated
#
# At 90 deg the observed sample spread is 80 counts, which at 24.5
# counts/deg is +/-3.3 deg of apparent noise -- wider than the 2.9 deg
# stop deadband. Closed-loop settling above ~75 deg will chatter.
# See SWING_USABLE_MAX_DEG below.
#
# TODO: 30 and 60 sweep-deg were not measured. The 15->45 and 45->75
# gaps are interpolated linearly across regions where the slope is known
# to change by 50%+. Measure them.

SWING_RAW_MIN = 1600     # CW end / start position
SWING_RAW_MAX = 21447     # CCW end

# ----------------------------------------------------------------
# REPORTING CONVENTION
# ----------------------------------------------------------------
# Which sweep position reports 0 rad on /joint_states and in trajectory
# goals. 0.0 => end-referenced: the start position is 0, angle increases
# CCW to +90. Set to 45.0 for a centred -45..+45 convention instead.
#
# CHANGING THIS CHANGES THE UNITS OF EVERY SWING TRAJECTORY GOAL.
# degrees.yaml and the URDF joint limits must be updated to match.
SWING_ZERO_DEG = 0.0

# ----------------------------------------------------------------
# MEASURED PHYSICAL TRAVEL (open-loop sweep, pwm 230)
# ----------------------------------------------------------------
# The joint actually reaches raw 8864..25440 -- about 189 deg of travel,
# roughly twice what SWING_RAW_TABLE covers. The calibrated band is a
# SOFT limit (where positioning is trusted), not the physical range.
#
# The reject window must span the PHYSICAL range, not the calibrated one.
# When it was derived from the table (8848..20736), 56% of the samples in
# a real sweep fell outside it and would have been discarded as glitches,
# freezing the reported position while the motor kept driving -- the exact
# runaway mechanism the watchdog exists to catch.
#
# TRADE-OFF: this window is now wide enough that it no longer catches the
# ~22800-count bus glitch seen earlier, because that value is inside the
# real travel. Glitch rejection now rests on the median-of-5 read and the
# progress watchdog, not on this range check.
SWING_RAW_PHYS_MIN = 1600
SWING_RAW_PHYS_MAX = 21447
SWING_RAW_MARGIN = 500
SWING_RAW_REJECT_LO = SWING_RAW_PHYS_MIN - SWING_RAW_MARGIN
SWING_RAW_REJECT_HI = SWING_RAW_PHYS_MAX + SWING_RAW_MARGIN

# ----------------------------------------------------------------
# SENSOR VALIDITY CEILING
# ----------------------------------------------------------------
# Above this raw count the photo-reflector is NOT single-valued. In the
# open-loop sweep, driving steadily in one direction produced reversals of
# -1280, -2832 and -272 going up, and +704, +240, +2640 coming down. Every
# reversal occurred above ~19000; below it the trace is clean and
# monotonic. Closed-loop positioning against a sensor that moves backwards
# while the joint moves forwards is not possible -- the controller will
# reverse, hunt, and trip the watchdog.
SWING_RAW_TRUSTED_MAX = 21447

# How many times to re-sample before giving up on a control cycle.
SWING_READ_RETRIES = 3


# ----------------------------------------------------------------
# RUNAWAY WATCHDOG
# ----------------------------------------------------------------
# The hard limits in plan_toward_target() are written in terms of the
# SEMANTIC direction (+1 = increase angle). That is correct once the sign
# convention is right, but it means they provide ZERO protection when the
# sign is wrong: commanding +1 while the motor physically drives raw down
# never satisfies "raw >= SWING_RAW_MAX", so the swing drives into the CW
# stop and stalls there. Same story for a frozen sensor -- a stale raw
# value never crosses either limit.
#
# These watchdogs close both holes by checking that the raw count is
# actually moving the way the commanded direction says it should.

# Consecutive cycles with no in-range sample before the swing is declared
# blind and refuses to move.
SWING_MAX_STALE_CYCLES = 5

# Cycles of continuous drive in one direction before progress is checked.
SWING_PROGRESS_WINDOW = 30

# Raw counts of progress required over that window, in the direction the
# controller intended. Must clear the sensor noise floor (~80 counts
# peak-to-peak at the flat top of the curve) or noise alone decides the
# outcome. At the flattest slope, 100 counts is ~4 deg of real motion --
# so this WILL false-trip on slow moves above 75 deg. That is a sensor
# problem, not a tuning problem; fix the gradient rather than lowering
# this below the noise floor.
SWING_MIN_PROGRESS_COUNTS = 100

# ----------------------------------------------------------------
# SWING PWM LADDER
# ----------------------------------------------------------------
# full_extest.py moves this joint at 220 and 255. The old ladder here
# topped out at 180 and gave 155 for a 10 deg error -- roughly 30% below
# the lowest speed the swing has ever been observed to move at. That is
# why the watchdog reported "NOT MOVING": the controller was commanding a
# duty the motor cannot break stiction at, so raw never changed.
#
# The gap between the slowest usable speed (~220) and full scale (255) is
# narrow, so there is very little proportional authority here. Expect
# coarse, near-bang-bang motion. If it overshoots, the fix is pulsing
# (short bursts at 220+ with gaps) rather than lowering these numbers
# back below the stiction threshold.
SWING_PWM_FAR   = 255   # err > 0.50 rad
SWING_PWM_MID   = 240   # err > 0.25 rad
SWING_PWM_NEAR  = 230   # err > 0.10 rad
SWING_PWM_FINAL = 220   # everything above the stop tolerance

SWING_RAW_TABLE = [
    1600,     # -90
    1800,     # -45°
    3955,     # -30°
    6968,     #   0°
    9943,     # +30°
    11410,    # +45°
    14282,    # +60°
    17497,    # +75°
    21447,    # +90°
]

SWING_SWEEP_DEG = [
    -90,
    -45.0,
    -30.0,
    0.0,
    30.0,
    45.0,
    60.0,
    75.0,
    90.0,
]

# Reported angle = sweep position - SWING_ZERO_DEG.
SWING_ANGLE_TABLE = [
    math.radians(d - SWING_ZERO_DEG) for d in SWING_SWEEP_DEG
]

# Derived travel limits in the reporting convention.
SWING_ANGLE_MIN_RAD = SWING_ANGLE_TABLE[0]
SWING_ANGLE_MAX_RAD = SWING_ANGLE_TABLE[-1]

# ----------------------------------------------------------------
# USABLE RANGE
# ----------------------------------------------------------------
# Above this sweep position the gradient is too flat to position against
# (24.5 counts/deg vs 138 lower down, with a noise floor of ~80 counts).
# Set to 75.0 to clamp goals into the region the sensor can actually
# resolve; set to 90.0 to allow the full travel and accept chatter there.
SWING_USABLE_MAX_DEG = 90.0
SWING_USABLE_MAX_RAD = math.radians(SWING_USABLE_MAX_DEG - SWING_ZERO_DEG)

# Fail loudly at import time rather than silently interpolating garbage.
assert len(SWING_RAW_TABLE) == len(SWING_SWEEP_DEG), \
    "SWING_RAW_TABLE and SWING_SWEEP_DEG must be the same length"
assert all(
    SWING_RAW_TABLE[i] < SWING_RAW_TABLE[i + 1]
    for i in range(len(SWING_RAW_TABLE) - 1)
), "SWING_RAW_TABLE must be strictly increasing"
assert all(
    SWING_SWEEP_DEG[i] < SWING_SWEEP_DEG[i + 1]
    for i in range(len(SWING_SWEEP_DEG) - 1)
), "SWING_SWEEP_DEG must be strictly increasing"
assert SWING_SWEEP_DEG[0] <= SWING_ZERO_DEG <= SWING_SWEEP_DEG[-1], \
    "SWING_ZERO_DEG must lie inside the calibrated travel"


def swing_raw_to_rad(raw: int) -> float:
    """Piecewise-linear lookup: raw ADC counts -> swing angle in radians."""
    if raw <= SWING_RAW_TABLE[0]:
        return SWING_ANGLE_TABLE[0]
    if raw >= SWING_RAW_TABLE[-1]:
        return SWING_ANGLE_TABLE[-1]

    for i in range(len(SWING_RAW_TABLE) - 1):
        raw0 = SWING_RAW_TABLE[i]
        raw1 = SWING_RAW_TABLE[i + 1]
        if raw0 <= raw <= raw1:
            a0 = SWING_ANGLE_TABLE[i]
            a1 = SWING_ANGLE_TABLE[i + 1]
            u = (raw - raw0) / float(raw1 - raw0)
            return a0 + u * (a1 - a0)

    # Unreachable given the strictly-increasing assertions above.
    return SWING_ANGLE_TABLE[-1]


def swing_rad_to_raw(angle_rad: float) -> int:
    """Inverse of swing_raw_to_rad(). Used to report the centre raw count."""
    if angle_rad <= SWING_ANGLE_TABLE[0]:
        return SWING_RAW_TABLE[0]
    if angle_rad >= SWING_ANGLE_TABLE[-1]:
        return SWING_RAW_TABLE[-1]

    for i in range(len(SWING_ANGLE_TABLE) - 1):
        a0 = SWING_ANGLE_TABLE[i]
        a1 = SWING_ANGLE_TABLE[i + 1]
        if a0 <= angle_rad <= a1:
            u = (angle_rad - a0) / float(a1 - a0)
            return int(round(
                SWING_RAW_TABLE[i]
                + u * (SWING_RAW_TABLE[i + 1] - SWING_RAW_TABLE[i])
            ))

    return SWING_RAW_TABLE[-1]


# Raw count at the reported zero. With SWING_ZERO_DEG=0 this equals
# SWING_RAW_MIN; it only diverges if the reporting zero is moved off the
# start position. Note this is the reported ZERO, not the centre of
# travel -- unlike the old adc_center=22380, which came from a previous
# sensor mounting and meant neither.
SWING_RAW_ZERO = swing_rad_to_raw(0.0)


@dataclass
class Waypoint:
    t: float
    positions: List[float]

# this function cleans the trajectory points before the robot follows them 
#traj point = time and positions 
def normalize_waypoints(
    points: Sequence[JointTrajectoryPoint], n_joints: int
) -> List[Waypoint]:
    out: List[Waypoint] = []
    last_t = -1e9
    for p in points:
        t = duration_to_seconds(p.time_from_start)
        if t < 0.0:
            t = 0.0 #force negative to 0 
        if t + 1e-9 < last_t: # make sure time does not go backward
            return []
        last_t = t
        if len(p.positions) != n_joints: # making sure each point has the correct number of joint values
            return []
        out.append(Waypoint(t=t, positions=list(p.positions)))

    if out and out[0].t > 1e-9: #if the first waypoint starts at 2 sec instead of 0, it sets a fake starting point at 0 using the same position. t= 2, boom = 0.5 -> t= 0, boom=0.5 then t=2, boom = 0.5
        out.insert(0, Waypoint(t=0.0, positions=list(out[0].positions)))
    return out


def interpolate_positions(waypoints: Sequence[Waypoint], t: float) -> List[float]: #this will answer, at this exact time, where should each joint be 
    if t <= waypoints[0].t:
        return list(waypoints[0].positions)
    if t >= waypoints[-1].t:
        return list(waypoints[-1].positions)

    i = 0
    while i + 1 < len(waypoints) and waypoints[i + 1].t < t:
        i += 1

    w0, w1 = waypoints[i], waypoints[i + 1]
    if abs(w1.t - w0.t) < 1e-12:
        return list(w1.positions)

    a = (t - w0.t) / (w1.t - w0.t)
    return [p0 + a * (p1 - p0) for p0, p1 in zip(w0.positions, w1.positions)]


# ================================================================
# ── SIM MODE (Isaac Sim / ROS computer) ─────────────────────────
# ================================================================

class SimTrajectoryServer(Node):
    """
    Trajectory bridge for Isaac Sim / ROS desktop.
    No pigpio, no GPIO, no ADS1115.
    Publishes interpolated JointState to /joint_command.
    """

    def __init__(self) -> None:
        super().__init__("excavator_trajectory_server_sim")

        self.declare_parameter("action_name",       "/upper_arm_controller/follow_joint_trajectory")
        self.declare_parameter("command_topic",     "/joint_command")
        self.declare_parameter("publish_hz",        60.0)
        self.declare_parameter("hold_last_seconds", 0.30)
        self.declare_parameter("feedback_hz",       10.0)
        self.declare_parameter("reliable",          True)
        self.declare_parameter("qos_depth",         10)

        action_name       = str(self.get_parameter("action_name").value)
        command_topic     = str(self.get_parameter("command_topic").value)
        self._publish_hz  = float(self.get_parameter("publish_hz").value) or 60.0
        self._hold_last   = float(self.get_parameter("hold_last_seconds").value)
        self._feedback_hz = float(self.get_parameter("feedback_hz").value) or 10.0
        reliable          = bool(self.get_parameter("reliable").value)
        qos_depth         = int(self.get_parameter("qos_depth").value) or 10

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
        )
        self._pub = self.create_publisher(JointState, command_topic, qos)

        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            action_name,
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
        )

        self.get_logger().info("[SIM MODE] ExcavatorTrajectoryServer ready")
        self.get_logger().info(f"  Action  : {action_name}")
        self.get_logger().info(f"  Publishes: {command_topic} (sensor_msgs/JointState)")

    def _goal_cb(self, goal_request):
        if not goal_request.trajectory.joint_names:
            return GoalResponse.REJECT
        if not goal_request.trajectory.points:
            return GoalResponse.REJECT
        n = len(goal_request.trajectory.joint_names)
        for i, p in enumerate(goal_request.trajectory.points):
            if len(p.positions) not in (0, n):
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        self.get_logger().info("[SIM] Cancel received")
        return CancelResponse.ACCEPT

    def _execute_cb(self, goal_handle) -> FollowJointTrajectory.Result:
        goal = goal_handle.request
        joint_names = list(goal.trajectory.joint_names)
        waypoints = normalize_waypoints(goal.trajectory.points, len(joint_names))

        result = FollowJointTrajectory.Result()
        if not waypoints:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "No valid waypoints"
            return result

        duration = waypoints[-1].t
        dt = 1.0 / self._publish_hz
        fb_dt = 1.0 / self._feedback_hz

        self.get_logger().info(
            f"[SIM] Executing: joints={joint_names}, points={len(waypoints)}, duration={duration:.2f}s"
        )

        js = JointState()
        js.name = joint_names
        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = joint_names

        start = time.monotonic()
        next_fb = 0.0
        last_positions = waypoints[0].positions
        t = 0.0

        while t <= duration + 1e-9:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = "Canceled"
                return result

            positions = interpolate_positions(waypoints, t)
            last_positions = positions

            js.header.stamp = self.get_clock().now().to_msg()
            js.position = positions
            self._pub.publish(js)

            if t >= next_fb:
                desired = JointTrajectoryPoint()
                desired.positions = positions
                desired.time_from_start = seconds_to_duration(t)
                feedback.desired = desired
                goal_handle.publish_feedback(feedback)
                next_fb += fb_dt

            _sleep_until(start, t + dt)
            t = time.monotonic() - start

        # Hold last position
        if self._hold_last > 0.0:
            hold_end = time.monotonic() + self._hold_last
            while time.monotonic() < hold_end:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = "Canceled"
                    self.get_logger().info("[SIM] Trajectory canceled during hold")
                    return result
                js.header.stamp = self.get_clock().now().to_msg()
                js.position = last_positions
                self._pub.publish(js)
                time.sleep(dt)

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "OK"
        self.get_logger().info("[SIM] Trajectory done")
        return result


# ================================================================
# ── PI MODE (Raspberry Pi with pigpio + ADS1115) ─────────────────
# ================================================================

def _import_pi_libs():
    global pigpio, board, busio, ADS_mod, ADS1115_cls, AnalogIn_cls, Pin_cls
    import pigpio as pigpio  # noqa: F401

    try:
        import board as board
        import busio as busio
        import adafruit_ads1x15.ads1115 as ADS_mod
        from adafruit_ads1x15.ads1115 import ADS1115 as ADS1115_cls
        from adafruit_ads1x15.analog_in import AnalogIn as AnalogIn_cls
        from adafruit_ads1x15.ads1x15 import Pin as Pin_cls
    except ImportError:
        board = busio = ADS_mod = ADS1115_cls = AnalogIn_cls = Pin_cls = None


class ADS1115Reader:
    def __init__(self, address: int = 0x48, gain: int = 1):
        if ADS_mod is None:
            raise RuntimeError(
                "ADS1115 libraries not installed. Run: pip3 install adafruit-circuitpython-ads1x15"
            )
        i2c = busio.I2C(board.SCL, board.SDA)
        self.ads = ADS1115_cls(i2c, address=address)
        self.ads.gain = gain
        self.ads.data_rate = 860
        self.channels = {
            0: AnalogIn_cls(self.ads, Pin_cls.A0),
            1: AnalogIn_cls(self.ads, Pin_cls.A1),
            2: AnalogIn_cls(self.ads, Pin_cls.A2),
            3: AnalogIn_cls(self.ads, Pin_cls.A3),
        }
        self._i2c_lock = threading.Lock()
    def read(self, channel: int) -> int: # gets the raw value 
        with self._i2c_lock:
            return int(self.channels[channel].value)

    def read_median3(self, channel: int) -> int: # 5 samples, take the middle one
        with self._i2c_lock:
            s = [int(self.channels[channel].value) for _ in range(5)]
        return sorted(s)[2]

    def voltage(self, channel: int) -> float: # gets the voltage value 
        with self._i2c_lock:
            return float(self.channels[channel].voltage)

    def close(self) -> None:
        """ADS1115 uses the shared I2C bus; no explicit close is required here."""
        return None


@dataclass
class PotJointConfig: # holds config values
    name: str
    in1_pin: int
    in2_pin: int
    pwm_pin: int
    limit_positive: Optional[int] = None
    limit_negative: Optional[int] = None
    adc_channel: int = 0
    adc_min: int = 3000
    adc_max: int = 26000
    adc_center: Optional[int] = None
    angle_min_rad: float = -1.0
    angle_max_rad: float = 1.0
    invert_pot: bool = False
    invert_motor: bool = False
    tolerance_rad: float = 0.035
    stop_tolerance_rad: float = 0.020
    kp: float = 180.0
    min_pwm: int = 90
    max_pwm: int = 180
    kick_pwm: int = 170
    pwm_frequency_hz: int = 1000
    # Pulsed approach. Below pulse_err_rad the joint drives for
    # pulse_on_cycles then coasts for pulse_off_cycles, lowering the
    # AVERAGE speed while keeping instantaneous duty above stiction.
    # Needed when min_pwm is high enough that one control cycle of drive
    # covers more than the stop tolerance. 0 off-cycles disables it.
    pulse_err_rad: float = 0.0
    pulse_on_cycles: int = 1
    pulse_off_cycles: int = 0


class PotentiometerJointMotor: #preparing everything the controller will need. which also contains the above class. 
    def __init__(self, pi, adc: ADS1115Reader, cfg: PotJointConfig):
        self.pi = pi
        self.adc = adc
        self.cfg = cfg
        self.name = cfg.name
        self._last_position_rad = 0.0
        self._last_velocity_rad_s = 0.0
        self._last_time = time.monotonic()
        self._lock = threading.Lock()
        self._holding_target = False
        self._last_target_rad = None
        self._target_stable_since = time.monotonic()
        self._last_valid_raw = None
        # False until at least one in-range sample has been read. The
        # controller refuses to drive the swing while this is False.
        self._swing_raw_valid = False
        self._swing_stale_cycles = 0
        # Raw count when the swing is parked outside SWING_RAW_MIN..MAX,
        # where the reported angle is a clamped endpoint and not a real
        # measurement. None when in band.
        self._swing_out_of_cal = None
        self._swing_pulse_tick = 0
        # Runaway watchdog state
        self._swing_watch_dir = 0
        self._swing_watch_raw = None
        self._swing_watch_cycles = 0
        self._swing_fault = None
        self.requested_pwm = 0

        # Debug trace: keep both the target received by the controller and
        # the target after any local clamping. This makes it possible to see
        # exactly where a commanded 0 deg might turn into another value.
        self._last_requested_target_rad = None
        self._last_effective_target_rad = None

        pi.set_mode(cfg.in1_pin, pigpio.OUTPUT)
        pi.set_mode(cfg.in2_pin, pigpio.OUTPUT)
        pi.set_mode(cfg.pwm_pin, pigpio.OUTPUT)
        pi.set_PWM_frequency(cfg.pwm_pin, cfg.pwm_frequency_hz)
        pi.set_PWM_dutycycle(cfg.pwm_pin, 0)

        if cfg.limit_positive is not None: # probably do not need to worry about this much. we have the pots to do it for us. 
            pi.set_mode(cfg.limit_positive, pigpio.INPUT)
            pi.set_pull_up_down(cfg.limit_positive, pigpio.PUD_UP)
        if cfg.limit_negative is not None:
            pi.set_mode(cfg.limit_negative, pigpio.INPUT)
            pi.set_pull_up_down(cfg.limit_negative, pigpio.PUD_UP)
    def read_position_rad(self) -> float:
        c = self.cfg

        # ==========================================================
        # SWING
        # ==========================================================
        if c.name == "swing_joint":

            # Read a trustworthy swing ADC sample
            raw = None

            for _ in range(SWING_READ_RETRIES):

                candidate = self.adc.read_median3(c.adc_channel)

                # Reject only values completely outside the physical sensor range
                if candidate < SWING_RAW_REJECT_LO or candidate > SWING_RAW_REJECT_HI:
                    continue



                # Candidate passed all checks
                raw = candidate
                break

            if raw is None:
                self._swing_stale_cycles += 1

                if (self._last_valid_raw is None
                        or self._swing_stale_cycles > SWING_MAX_STALE_CYCLES):
                    # Either no trustworthy sample has EVER arrived, or the
                    # sensor has gone quiet while we were driving. The old
                    # code substituted the table minimum in the first case and held the
                    # last value forever in the second -- both let the motor
                    # keep driving on a position that is not real.
                    self._swing_raw_valid = False
                    with self._lock:
                        self._last_velocity_rad_s = 0.0
                        self._last_time = time.monotonic()
                    return self._last_position_rad

                # Brief glitch. Hold the last good sample for this cycle.
                raw = self._last_valid_raw
            else:
                self._last_valid_raw = raw
                self._swing_stale_cycles = 0

            self._swing_raw_valid = True

            # The lookup clamps out-of-band raw to the table endpoints, so a
            # joint parked below SWING_RAW_MIN reports exactly 0.0 rad and
            # looks perfectly healthy. Record the real raw so the controller
            # can say so rather than acting on a clamped reading.
            if raw < SWING_RAW_MIN or raw > SWING_RAW_MAX:
                self._swing_out_of_cal = raw
            else:
                self._swing_out_of_cal = None

            angle = swing_raw_to_rad(raw)

        # ==========================================================
        # OTHER JOINTS WITH CENTER
        # ==========================================================
        elif c.adc_center is not None:
            raw = self.adc.read(c.adc_channel)

            if raw <= c.adc_center:
                span = max(1, c.adc_center - c.adc_min)

                u = clamp(
                    (raw - c.adc_min) / span,
                    0.0,
                    1.0,
                )

                angle = c.angle_min_rad + u * (
                    0.0 - c.angle_min_rad
                )

            else:
                span = max(1, c.adc_max - c.adc_center)

                u = clamp(
                    (raw - c.adc_center) / span,
                    0.0,
                    1.0,
                )

                angle = u * c.angle_max_rad

        # ==========================================================
        # NORMAL POTENTIOMETERS
        # ==========================================================
        else:
            raw = self.adc.read(c.adc_channel)

            span = max(1, c.adc_max - c.adc_min)

            u = clamp(
                (raw - c.adc_min) / span,
                0.0,
                1.0,
            )

            if c.invert_pot:
                u = 1.0 - u

            angle = c.angle_min_rad + u * (
                c.angle_max_rad - c.angle_min_rad
            )

        # ==========================================================
        # VELOCITY
        # THIS MUST BE OUTSIDE ALL THREE ANGLE-CONVERSION BRANCHES
        # ==========================================================
        now = time.monotonic()
        dt = max(1e-6, now - self._last_time)

        with self._lock:
            self._last_velocity_rad_s = (
                angle - self._last_position_rad
            ) / dt

            self._last_position_rad = angle
            self._last_time = now

        return angle

    def read_velocity_rad_s(self) -> float:
        with self._lock:
            return float(self._last_velocity_rad_s)

    def positive_limit_reached(self) -> bool:
        return self.cfg.limit_positive is not None and self.pi.read(self.cfg.limit_positive) == 0

    def negative_limit_reached(self) -> bool:
        return self.cfg.limit_negative is not None and self.pi.read(self.cfg.limit_negative) == 0

    def stop(self) -> None:
        self.pi.write(self.cfg.in1_pin, 0)
        self.pi.write(self.cfg.in2_pin, 0)
        self.requested_pwm = 0


    def drive_direction(self, desired_direction: int, pwm: int) -> None:
        
        if desired_direction == 0:
            self.stop()
            return
        if desired_direction > 0 and self.positive_limit_reached():
            self.stop()
            return
        if desired_direction < 0 and self.negative_limit_reached():
            self.stop()
            return

        motor_dir = -desired_direction if self.cfg.invert_motor else desired_direction
        pwm = int(clamp(pwm, 0, 255))
        if motor_dir > 0:
            self.pi.write(self.cfg.in1_pin, 1)
            self.pi.write(self.cfg.in2_pin, 0)
        else:
            self.pi.write(self.cfg.in1_pin, 0)
            self.pi.write(self.cfg.in2_pin, 1)
        self.requested_pwm = pwm
    def plan_toward_target(
        self, target_rad: float
    ) -> Tuple[float, float, bool, int, int]:
        """
        Decide what this joint WANTS to do, without touching the motor.

        Returns (pos, err, at_goal, direction, pwm).

        `direction` is semantic: +1 means "increase the joint angle",
        -1 means "decrease it". drive_direction() maps that onto the
        physical pins and applies cfg.invert_motor, so the hard limits
        below stay correct whichever way invert_motor is set.

        Split out from update_toward_target() so the trajectory loop can
        collect every joint's intent first and then arbitrate the single
        shared PWM pin -- see PiExcavatorTrajectoryServer._apply_plans().
        """
        pos = self.read_position_rad()

        # ==========================================================
        # SWING - SIMPLE CLOSED LOOP
        # ==========================================================
        if self.cfg.name == "swing_joint":

            # Preserve the exact target passed into this controller before
            # applying any safety clamp. These values are only for diagnosis.
            self._last_requested_target_rad = target_rad

            target_rad = clamp(
                target_rad,
                self.cfg.angle_min_rad,
                min(self.cfg.angle_max_rad, SWING_USABLE_MAX_RAD),
            )
            self._last_effective_target_rad = target_rad

            err = target_rad - pos
            abs_err = abs(err)

            # Latched fault: something is physically wrong. Stay stopped
            # until reset_swing_fault() is called.
            if self._swing_fault is not None:
                self._swing_watch_dir = 0
                return pos, err, False, 0, 0

            # No trustworthy sensor reading -> do not move.
            if not self._swing_raw_valid:
                self._swing_watch_dir = 0
                return pos, err, False, 0, 0

            # Close enough
            if abs_err <= self.cfg.stop_tolerance_rad:
                self._swing_watch_dir = 0
                return pos, err, True, 0, 0

            # Direction from CURRENT error only
            direction = 1 if err > 0.0 else -1

            raw_now = self._last_valid_raw

            # HARD SAFETY LIMITS. These only work once the sign convention
            # is correct -- see the watchdog below for the case where it
            # is not.
            if direction > 0 and raw_now >= SWING_RAW_MAX:
                self._swing_watch_dir = 0
                return pos, err, True, 0, 0

            if direction < 0 and raw_now <= SWING_RAW_MIN:
                self._swing_watch_dir = 0
                return pos, err, True, 0, 0

            # SENSOR VALIDITY CEILING. Above SWING_RAW_TRUSTED_MAX the
            # photo-reflector is not single-valued, so the reported angle
            # is meaningless. Allow driving back down out of that region,
            # never further into it.
            if raw_now >= SWING_RAW_TRUSTED_MAX and direction > 0:
                self._swing_watch_dir = 0
                self._swing_fault = (
                    f"swing at raw {raw_now}, above the trusted ceiling "
                    f"{SWING_RAW_TRUSTED_MAX}. The photo-reflector is not "
                    f"single-valued up here; position cannot be trusted. "
                    f"Jog it back down below {SWING_RAW_TRUSTED_MAX}."
                )
                return pos, err, False, 0, 0

            # ------------------------------------------------------
            # PULSED APPROACH
            # ------------------------------------------------------
            # At 220+ PWM this joint covers ~3.4 deg per 20 ms control
            # cycle -- more than the whole 2.9 deg stop tolerance. It
            # literally cannot stop inside the deadband, so it overshoots,
            # reverses, overshoots back: the shaking.
            #
            # Lowering the PWM is not an option (below ~220 it does not
            # move at all), so lower the DUTY IN TIME instead: drive one
            # cycle, coast several. Instantaneous torque stays above
            # stiction, average speed drops by the on/off ratio.
            #
            # Returns pwm 0 while preserving `direction`, so the pins stay
            # set and the watchdog window is not reset by coast cycles.
            if (self.cfg.pulse_off_cycles > 0
                    and abs_err <= self.cfg.pulse_err_rad):
                period = self.cfg.pulse_on_cycles + self.cfg.pulse_off_cycles
                phase = self._swing_pulse_tick % period
                self._swing_pulse_tick += 1
                if phase >= self.cfg.pulse_on_cycles:
                    return pos, err, False, direction, 0
            else:
                self._swing_pulse_tick = 0

            # ------------------------------------------------------
            # RUNAWAY WATCHDOG
            # ------------------------------------------------------
            # raw is defined to increase with the reported angle, so after
            # a window of continuous drive, (raw_now - raw_at_window_start)
            # must have the same sign as `direction` and some magnitude.
            # Wrong invert_motor gives a negative product; a stall or a
            # frozen sensor gives roughly zero. Both latch a fault.
            if direction != self._swing_watch_dir:
                self._swing_watch_dir = direction
                self._swing_watch_raw = raw_now
                self._swing_watch_cycles = 0
            else:
                self._swing_watch_cycles += 1

                if self._swing_watch_cycles >= SWING_PROGRESS_WINDOW:
                    progress = (raw_now - self._swing_watch_raw) * direction

                    if progress < SWING_MIN_PROGRESS_COUNTS:
                        if progress < -SWING_MIN_PROGRESS_COUNTS:
                            self._swing_fault = (
                                f"swing moving BACKWARDS: commanded direction "
                                f"{direction:+d} but raw went "
                                f"{self._swing_watch_raw} -> {raw_now} over "
                                f"{SWING_PROGRESS_WINDOW} cycles. "
                                f"invert_motor is almost certainly wrong."
                            )
                        else:
                            self._swing_fault = (
                                f"swing NOT MOVING: commanded direction "
                                f"{direction:+d}, raw stuck near {raw_now} over "
                                f"{SWING_PROGRESS_WINDOW} cycles. "
                                f"Stalled against a stop, PWM too low, or the "
                                f"photo-reflector is not tracking."
                            )
                        self._swing_watch_dir = 0
                        return pos, err, False, 0, 0

                    # Progress is real. Start a fresh window.
                    self._swing_watch_raw = raw_now
                    self._swing_watch_cycles = 0

            # Swing PWM ladder. This only has any effect now that the
            # shared PWM pin is arbitrated -- previously max() across all
            # joints overrode it on any multi-joint trajectory.
            if abs_err > 0.50:
                pwm = SWING_PWM_FAR
            elif abs_err > 0.25:
                pwm = SWING_PWM_MID
            elif abs_err > 0.10:
                pwm = SWING_PWM_NEAR
            else:
                pwm = SWING_PWM_FINAL

            return pos, err, False, direction, pwm

        # ==========================================================
        # BOOM / ARM / BUCKET
        # ==========================================================
        err = target_rad - pos
        abs_err = abs(err)

        if abs_err <= self.cfg.stop_tolerance_rad:
            return pos, err, True, 0, 0

        direction = 1 if err > 0.0 else -1

        pwm = int(
            clamp(
                abs_err * self.cfg.kp,
                self.cfg.min_pwm,
                self.cfg.max_pwm,
            )
        )

        if abs_err > 0.20 and pwm < self.cfg.kick_pwm:
            pwm = min(
                self.cfg.kick_pwm,
                self.cfg.max_pwm,
            )

        return pos, err, abs_err <= self.cfg.tolerance_rad, direction, pwm

    def reset_swing_fault(self) -> None:
        """Clear a latched watchdog fault. Fix the cause first."""
        self._swing_fault = None
        self._swing_watch_dir = 0
        self._swing_watch_raw = None
        self._swing_watch_cycles = 0

    def apply_plan(self, direction: int, pwm: int) -> None:
        """Commit a plan produced by plan_toward_target()."""
        if direction == 0:
            self.stop()
        else:
            self.drive_direction(direction, pwm)

    def update_toward_target(self, target_rad: float) -> Tuple[float, float, bool]:
        """Single-joint convenience wrapper. Does NOT set the shared PWM pin."""
        pos, err, at_goal, direction, pwm = self.plan_toward_target(target_rad)
        self.apply_plan(direction, pwm)
        return pos, err, at_goal



@dataclass # most likely not going to use the open looped classes. closed loop is better and open loop is used when we dont have a potiometer or photorefl
class OpenLoopJointConfig:
    name: str
    in1_pin: int
    in2_pin: int
    pwm_pin: int
    angle_min_rad: float = -1.57
    angle_max_rad: float = 1.57
    invert_motor: bool = False
    tolerance_rad: float = 0.04
    kp: float = 180.0
    min_pwm: int = 90
    max_pwm: int = 160
    estimated_speed_rad_s_at_max_pwm: float = 0.80


class OpenLoopEstimatedJointMotor: # can disregard the open loop stuff, maybe we will use it again 
    def __init__(self, pi, cfg: OpenLoopJointConfig):
        self.pi = pi
        self.cfg = cfg
        self.name = cfg.name
        self._position_rad = 0.0
        self._velocity_rad_s = 0.0
        self._last_update = time.monotonic()
        self._lock = threading.Lock()

        pi.set_mode(cfg.in1_pin, pigpio.OUTPUT)
        pi.set_mode(cfg.in2_pin, pigpio.OUTPUT)
        pi.set_mode(cfg.pwm_pin, pigpio.OUTPUT)
        pi.set_PWM_frequency(cfg.pwm_pin, 1000)
        pi.set_PWM_dutycycle(cfg.pwm_pin, 0)

        pi.write(cfg.in1_pin, 0)
        pi.write(cfg.in2_pin, 0)
        pi.set_PWM_dutycycle(cfg.pwm_pin, 0)

    def _integrate_estimate(self) -> None:
        now = time.monotonic()
        dt = max(1e-6, now - self._last_update)
        with self._lock:
            self._position_rad = clamp(
                self._position_rad + self._velocity_rad_s * dt,
                self.cfg.angle_min_rad,
                self.cfg.angle_max_rad,
            )
            self._last_update = now

    def read_position_rad(self) -> float:
        self._integrate_estimate()
        with self._lock:
            return float(self._position_rad)

    def read_velocity_rad_s(self) -> float:
        with self._lock:
            return float(self._velocity_rad_s)

    def stop(self) -> None:
        self._integrate_estimate()
        self.pi.write(self.cfg.in1_pin, 0)
        self.pi.write(self.cfg.in2_pin, 0)
        
        with self._lock:
            self._velocity_rad_s = 0.0

    def update_toward_target(self, target_rad: float) -> Tuple[float, float, bool]:
        pos = self.read_position_rad()
        target_rad = clamp(target_rad, self.cfg.angle_min_rad, self.cfg.angle_max_rad)
        err = target_rad - pos

        if abs(err) <= self.cfg.tolerance_rad:
            self.stop()
            return pos, err, True

        desired_dir = 1 if err > 0.0 else -1
        motor_dir = -desired_dir if self.cfg.invert_motor else desired_dir
        pwm = int(clamp(abs(err) * self.cfg.kp, self.cfg.min_pwm, self.cfg.max_pwm))

        if motor_dir > 0:
            self.pi.write(self.cfg.in1_pin, 1)
            self.pi.write(self.cfg.in2_pin, 0)
        else:
            self.pi.write(self.cfg.in1_pin, 0)
            self.pi.write(self.cfg.in2_pin, 1)
        self.pi.set_PWM_dutycycle(self.cfg.pwm_pin, pwm)

        speed_ratio = pwm / max(1, self.cfg.max_pwm)
        with self._lock:
            self._velocity_rad_s = desired_dir * self.cfg.estimated_speed_rad_s_at_max_pwm * speed_ratio

        return pos, err, False


@dataclass
class PhotoReflectorConfig:
    name: str
    adc_channel: int
    adc_min: int
    adc_max: int
    angle_min_rad: float
    angle_max_rad: float
    invert_sensor: bool = False 
    #adc_channel: int = 3
    #threshold_raw: int = 16000
    #active_when_above_threshold: bool = True


class PhotoReflector:
    def __init__(self, adc: ADS1115Reader, cfg: PhotoReflectorConfig):
        self.adc = adc
        self.cfg = cfg

    def raw(self) -> int:
        return self.adc.read(self.cfg.adc_channel)
        
    def voltage(self) -> float:
        return self.adc.voltage_(self.cfg.adc_channel)
    
    def position_rad(self) ->float:
        raw = self.raw()
        c = self.cfg
    
        span = max(1, c.adc_max - c.adc_min)
        u = clamp((raw - c.adc_min) / span, 0.0, 1.0)
        
        if c.invert_sensor:
            u = 1.0 - u
        
        return c.angle_min_rad + u * (c.angle_max_rad - c.angle_min_rad)

    #def active(self) -> bool:
        #value = self.raw()
        # return value >= self.cfg.threshold_raw if self.cfg.active_when_above_threshold else value <= self.cfg.threshold_raw


class PiExcavatorTrajectoryServer(Node):
    """
    Real-robot trajectory server for Raspberry Pi.

    Hardware pins, ADC channels, calibration, home position, and most control
    tuning values are loaded from the excavator YAML configuration.
    """

    def __init__(self, config_path: str) -> None:
        super().__init__("pi_excavator_trajectory_server")

        self.config_path = str(Path(config_path).expanduser().resolve())
        self.excavator_config = load_excavator_config(self.config_path)
        runtime_yaml = _load_runtime_yaml(self.config_path)

        gpio = _require_mapping(runtime_yaml, "gpio")
        control = _require_mapping(runtime_yaml, "control")
        home = _require_mapping(runtime_yaml, "home")
        joint_control = _require_mapping(runtime_yaml, "joint_control")

        _configure_swing_from_yaml(self.excavator_config)

        # YAML values are defaults. ROS parameters can still override them.
        self.declare_parameter(
            "action_name",
            "/upper_arm_controller/follow_joint_trajectory",
        )
        self.declare_parameter("publish_hz", float(control.get("publish_hz", 10.0)))
        self.declare_parameter("control_hz", float(control.get("control_hz", 50.0)))
        self.declare_parameter("feedback_hz", float(control.get("feedback_hz", 10.0)))
        self.declare_parameter(
            "goal_tolerance_rad",
            float(control.get("goal_tolerance_rad", 0.12)),
        )
        self.declare_parameter(
            "stop_on_goal_finish",
            bool(control.get("stop_on_goal_finish", True)),
        )
        self.declare_parameter(
            "ads1115_address",
            int(control.get("ads1115_address", 0x48)),
        )
        self.declare_parameter(
            "pwm_arbitration",
            str(control.get("pwm_arbitration", "exclusive")),
        )

        swing_gpio = _require_mapping(gpio, "swing")
        self.declare_parameter(
            "swing_invert_motor",
            bool(swing_gpio.get("invert_motor", False)),
        )

        swing_control = _require_mapping(joint_control, "swing")
        self.declare_parameter(
            "swing_pulse_err_deg",
            float(swing_control.get("pulse_err_deg", 5.0)),
        )
        self.declare_parameter(
            "swing_pulse_on",
            int(swing_control.get("pulse_on_cycles", 1)),
        )
        self.declare_parameter(
            "swing_pulse_off",
            int(swing_control.get("pulse_off_cycles", 2)),
        )

        self.action_name = str(self.get_parameter("action_name").value)
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.control_hz = float(self.get_parameter("control_hz").value)
        self.feedback_hz = float(self.get_parameter("feedback_hz").value)
        self.goal_tolerance_rad = float(
            self.get_parameter("goal_tolerance_rad").value
        )
        self.stop_on_goal_finish = bool(
            self.get_parameter("stop_on_goal_finish").value
        )
        ads_addr = int(self.get_parameter("ads1115_address").value)
        self.pwm_arbitration = str(
            self.get_parameter("pwm_arbitration").value
        ).strip().lower()
        swing_invert_motor = bool(
            self.get_parameter("swing_invert_motor").value
        )
        swing_pulse_err_rad = math.radians(
            float(self.get_parameter("swing_pulse_err_deg").value)
        )
        swing_pulse_on = max(
            1,
            int(self.get_parameter("swing_pulse_on").value),
        )
        swing_pulse_off = max(
            0,
            int(self.get_parameter("swing_pulse_off").value),
        )

        if self.pwm_arbitration not in ("exclusive", "max"):
            self.get_logger().warn(
                f"Unknown pwm_arbitration={self.pwm_arbitration!r}, "
                "using 'exclusive'"
            )
            self.pwm_arbitration = "exclusive"

        # Home targets are human-readable degrees in YAML.
        self.home_position = {
            "boom_joint": math.radians(float(home["boom_deg"])),
            "arm_joint": math.radians(float(home["arm_deg"])),
            "bucket_joint": math.radians(float(home["bucket_deg"])),
        }

        # Safety limits used to validate every FollowJointTrajectory goal
        # before the hardware is allowed to move. Values come directly from
        # excavator1.yaml through config_loader.py.
        self.joint_limits_rad = {
            "swing_joint": (
                math.radians(float(self.excavator_config.swing.min_angle_deg)),
                math.radians(float(self.excavator_config.swing.max_angle_deg)),
            ),
            "boom_joint": (
                math.radians(float(self.excavator_config.boom.min_angle_deg)),
                math.radians(float(self.excavator_config.boom.max_angle_deg)),
            ),
            "arm_joint": (
                math.radians(float(self.excavator_config.arm.min_angle_deg)),
                math.radians(float(self.excavator_config.arm.max_angle_deg)),
            ),
            "bucket_joint": (
                math.radians(float(self.excavator_config.bucket.min_angle_deg)),
                math.radians(float(self.excavator_config.bucket.max_angle_deg)),
            ),
        }

        for joint_name, (lower, upper) in self.joint_limits_rad.items():
            if lower > upper:
                raise RuntimeError(
                    f"Invalid configured limits for {joint_name}: "
                    f"{math.degrees(lower):.3f} deg > "
                    f"{math.degrees(upper):.3f} deg"
                )

        # Connect to pigpiod.
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError(
                "Failed to connect to pigpio daemon. Run: sudo pigpiod"
            )

        self.stby_pin = int(gpio["standby_pin"])
        self.shared_pwm_pin = int(gpio["shared_pwm_pin"])
        pwm_frequency_hz = int(control.get("pwm_frequency_hz", 1000))

        self.pi.set_mode(self.stby_pin, pigpio.OUTPUT)
        self.pi.write(self.stby_pin, 0)

        self.adc = ADS1115Reader(address=ads_addr)

        boom_gpio = _require_mapping(gpio, "boom")
        arm_gpio = _require_mapping(gpio, "arm")
        bucket_gpio = _require_mapping(gpio, "bucket")

        boom_control = _require_mapping(joint_control, "boom")
        arm_control = _require_mapping(joint_control, "arm")
        bucket_control = _require_mapping(joint_control, "bucket")

        boom_adc_min, boom_adc_max, boom_angle_at_min, boom_angle_at_max = (
            _linear_pot_endpoints(self.excavator_config.boom)
        )
        arm_adc_min, arm_adc_max, arm_angle_at_min, arm_angle_at_max = (
            _linear_pot_endpoints(self.excavator_config.arm)
        )
        bucket_adc_min, bucket_adc_max, bucket_angle_at_min, bucket_angle_at_max = (
            _linear_pot_endpoints(self.excavator_config.bucket)
        )

        self.joints: Dict[str, object] = {
            "swing_joint": PotentiometerJointMotor(
                self.pi,
                self.adc,
                PotJointConfig(
                    name="swing_joint",
                    in1_pin=int(swing_gpio["in1_pin"]),
                    in2_pin=int(swing_gpio["in2_pin"]),
                    pwm_pin=self.shared_pwm_pin,
                    adc_channel=int(self.excavator_config.swing.adc_channel),
                    adc_min=SWING_RAW_MIN,
                    adc_center=SWING_RAW_ZERO,
                    adc_max=SWING_RAW_MAX,
                    angle_min_rad=SWING_ANGLE_MIN_RAD,
                    angle_max_rad=SWING_ANGLE_MAX_RAD,
                    invert_pot=False,
                    invert_motor=swing_invert_motor,
                    kp=float(swing_control.get("kp", 120.0)),
                    min_pwm=int(swing_control.get("min_pwm", 220)),
                    max_pwm=int(swing_control.get("max_pwm", 255)),
                    kick_pwm=int(swing_control.get("kick_pwm", 255)),
                    pwm_frequency_hz=pwm_frequency_hz,
                    tolerance_rad=float(
                        swing_control.get("tolerance_rad", 0.04)
                    ),
                    stop_tolerance_rad=float(
                        swing_control.get("stop_tolerance_rad", 0.05)
                    ),
                    pulse_err_rad=swing_pulse_err_rad,
                    pulse_on_cycles=swing_pulse_on,
                    pulse_off_cycles=swing_pulse_off,
                ),
            ),
            "boom_joint": PotentiometerJointMotor(
                self.pi,
                self.adc,
                PotJointConfig(
                    name="boom_joint",
                    in1_pin=int(boom_gpio["in1_pin"]),
                    in2_pin=int(boom_gpio["in2_pin"]),
                    pwm_pin=self.shared_pwm_pin,
                    adc_channel=int(self.excavator_config.boom.adc_channel),
                    adc_min=boom_adc_min,
                    adc_max=boom_adc_max,
                    angle_min_rad=boom_angle_at_min,
                    angle_max_rad=boom_angle_at_max,
                    invert_pot=False,
                    invert_motor=bool(boom_gpio.get("invert_motor", False)),
                    kp=float(boom_control.get("kp", 120.0)),
                    min_pwm=int(boom_control.get("min_pwm", 130)),
                    max_pwm=int(boom_control.get("max_pwm", 255)),
                    kick_pwm=int(boom_control.get("kick_pwm", 180)),
                    pwm_frequency_hz=pwm_frequency_hz,
                    tolerance_rad=float(boom_control.get("tolerance_rad", 0.035)),
                    stop_tolerance_rad=float(
                        boom_control.get("stop_tolerance_rad", 0.015)
                    ),
                ),
            ),
            "arm_joint": PotentiometerJointMotor(
                self.pi,
                self.adc,
                PotJointConfig(
                    name="arm_joint",
                    in1_pin=int(arm_gpio["in1_pin"]),
                    in2_pin=int(arm_gpio["in2_pin"]),
                    pwm_pin=self.shared_pwm_pin,
                    adc_channel=int(self.excavator_config.arm.adc_channel),
                    adc_min=arm_adc_min,
                    adc_max=arm_adc_max,
                    angle_min_rad=arm_angle_at_min,
                    angle_max_rad=arm_angle_at_max,
                    invert_pot=False,
                    invert_motor=bool(arm_gpio.get("invert_motor", False)),
                    kp=float(arm_control.get("kp", 120.0)),
                    min_pwm=int(arm_control.get("min_pwm", 130)),
                    max_pwm=int(arm_control.get("max_pwm", 220)),
                    kick_pwm=int(arm_control.get("kick_pwm", 180)),
                    pwm_frequency_hz=pwm_frequency_hz,
                    tolerance_rad=float(arm_control.get("tolerance_rad", 0.035)),
                    stop_tolerance_rad=float(
                        arm_control.get("stop_tolerance_rad", 0.015)
                    ),
                ),
            ),
            "bucket_joint": PotentiometerJointMotor(
                self.pi,
                self.adc,
                PotJointConfig(
                    name="bucket_joint",
                    in1_pin=int(bucket_gpio["in1_pin"]),
                    in2_pin=int(bucket_gpio["in2_pin"]),
                    pwm_pin=self.shared_pwm_pin,
                    adc_channel=int(self.excavator_config.bucket.adc_channel),
                    adc_min=bucket_adc_min,
                    adc_max=bucket_adc_max,
                    angle_min_rad=bucket_angle_at_min,
                    angle_max_rad=bucket_angle_at_max,
                    invert_pot=False,
                    invert_motor=bool(bucket_gpio.get("invert_motor", False)),
                    kp=float(bucket_control.get("kp", 70.0)),
                    min_pwm=int(bucket_control.get("min_pwm", 75)),
                    max_pwm=int(bucket_control.get("max_pwm", 255)),
                    kick_pwm=int(bucket_control.get("kick_pwm", 80)),
                    pwm_frequency_hz=pwm_frequency_hz,
                    tolerance_rad=float(bucket_control.get("tolerance_rad", 0.08)),
                    stop_tolerance_rad=float(
                        bucket_control.get("stop_tolerance_rad", 0.06)
                    ),
                ),
            ),
        }

        self._stop_all()
        time.sleep(0.1)
        self.pi.write(self.stby_pin, 1)
        self._stop_all()
            
            

        # ── Joint configuration ──────────────────────────────────────
        # Edit adc_min/adc_max and angle_min_rad/angle_max_rad after calibration.
      #  self.joints: Dict[str, object] = {
       #     "swing_joint": OpenLoopEstimatedJointMotor(
        #        self.pi,
        #        OpenLoopJointConfig(
        #            name="swing_joint",
        #            in1_pin=10, in2_pin=27, pwm_pin=18,
        #            angle_min_rad=-1.57, angle_max_rad=1.57,
        #        ),
        #    ),
        #    "boom_joint": PotentiometerJointMotor(
        #        self.pi, self.adc,
        #        PotJointConfig(
        #            name="boom_joint",
        #            in1_pin=9, in2_pin=22, pwm_pin=18,
        #            limit_positive=7, limit_negative=8,
        #            adc_channel=0, adc_min=3000, adc_max=26000,
        #            angle_min_rad=-0.70, angle_max_rad=0.70,
        #        ),
        #    ),
        #    "arm_joint": PotentiometerJointMotor(
        #        self.pi, self.adc,
        #        PotJointConfig(
        #            name="arm_joint",
        #            in1_pin=25, in2_pin=24, pwm_pin=18,
        #            adc_channel=1, adc_min=3000, adc_max=26000,
        #            angle_min_rad=0.00, angle_max_rad=1.30,
        #        ),
        #    ),
        #   "bucket_joint": PotentiometerJointMotor(
        #        self.pi, self.adc,
        #        PotJointConfig(
        #            name="bucket_joint",
        #            in1_pin=5, in2_pin=6, pwm_pin=18,
        #            adc_channel=2, adc_min=3000, adc_max=26000,
        #           angle_min_rad=-1.00, angle_max_rad=0.60,
        #       ),
        #    ),
        #}
        
        
        #keep the last 10 messages and use reliable delivery
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        #this creates a ros publisher and pushes the current joint states to ROS so it can see the joints position in radians 
        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", qos)
        # creates the server that listens for movement commands
        #actually run the command, decide if valid, and stop if canceled
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            self.action_name,
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
        )
        #publishing joint states 
        self.create_timer(1.0 / max(1.0, self.publish_hz), self._publish_joint_states)

        self.get_logger().info("[PI MODE] PiExcavatorTrajectoryServer ready")
        self.get_logger().info(f"  Action  : {self.action_name}")
        self.get_logger().info("  Publishes feedback: /joint_states")
        self.get_logger().warn("  Calibrate adc_min/adc_max for boom/arm/bucket before real operation!")
        self.get_logger().warn(
            "  swing_joint range: "
            f"{math.degrees(SWING_ANGLE_MIN_RAD):+.1f} deg (raw {SWING_RAW_MIN}) .. "
            f"{math.degrees(SWING_ANGLE_MAX_RAD):+.1f} deg (raw {SWING_RAW_MAX}), "
            f"0 deg = sweep {SWING_ZERO_DEG:.1f} (raw {SWING_RAW_ZERO})"
        )
        self.get_logger().warn(
            f"  swing reject window: {SWING_RAW_REJECT_LO}..{SWING_RAW_REJECT_HI} "
            f"(physical travel {SWING_RAW_PHYS_MIN}..{SWING_RAW_PHYS_MAX}), "
            f"sensor trusted below {SWING_RAW_TRUSTED_MAX}, "
            f"goals clamped to {SWING_USABLE_MAX_DEG:.0f} deg"
        )
        self.get_logger().warn(
            "  swing trajectory goals must use THIS convention - check degrees.yaml and the URDF"
        )
        self.move_to_home()
    # ── Action callbacks ─────────────────────────────────────────
    def move_to_home(self):
        self.get_logger().info("[HOME] Moving excavator to starting position...")

        control_dt = 1.0 / max(1.0, self.control_hz)

        home_start = time.monotonic()
        HOME_TIMEOUT = 5.0

        while rclpy.ok():

            if time.monotonic() - home_start > HOME_TIMEOUT:
                self._stop_all()
                self.get_logger().error("[HOME] Timeout - stopping all motors.")
                return

            plans = {}
            all_home = True

            for joint_name, target in self.home_position.items():

                pos, err, at_goal, direction, pwm = \
                    self.joints[joint_name].plan_toward_target(target)

                plans[joint_name] = (direction, pwm, err)

                if not at_goal:
                    all_home = False

                self.get_logger().info(
                    f"[HOME] {joint_name}: "
                    f"current={pos:.3f} "
                    f"target={target:.3f} "
                    f"error={err:.3f}"
                )

            if all_home:
                self._stop_all()
                self.get_logger().info("[HOME] Starting position reached.")
                return

            self._apply_plans(plans)

            time.sleep(control_dt)
    # this gets called when the client sends a trajectory. it answers "should i accept this trajectory"
    # once we (the client) sends a trajector of the states, it does things like "is there another trajectory running, are the names correct, etc"
    def _goal_cb(self, goal_request):
        """Validate and accept/reject an incoming hardware trajectory goal."""
        names = list(goal_request.trajectory.joint_names)
        points = goal_request.trajectory.points

        try:
            validate_joint_targets(
                joint_names=names,
                points=points,
                joint_limits_rad=self.joint_limits_rad,
            )
        except ExcavatorGoalValidationError as exc:
            self.get_logger().warn(
                f"[GOAL REJECTED] {exc}"
            )
            return GoalResponse.REJECT

        # ==========================================================
        # DEBUG TRACE: WHAT ARRIVED FROM THE CLIENT?
        # ==========================================================
        # FollowJointTrajectory positions are radians at this point.
        # Printing both rad and deg lets us detect a client-side
        # swing offset before the controller touches the command.
        self.get_logger().warn(
            f"[SWING DEBUG][RECEIVED] joints={names}, "
            f"points={len(points)}"
        )

        if "swing_joint" in names:
            swing_index = names.index("swing_joint")

            for point_index, point in enumerate(points):
                t_sec = duration_to_seconds(point.time_from_start)
                swing_rad = float(point.positions[swing_index])

                self.get_logger().warn(
                    f"[SWING DEBUG][RECEIVED] "
                    f"point={point_index} "
                    f"t={t_sec:.3f}s "
                    f"target={swing_rad:+.6f} rad "
                    f"({math.degrees(swing_rad):+.2f} deg)"
                )

        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle): # running ctrl c or we say to cancel
        self.get_logger().warn("[PI] Cancel requested — stopping motors")
        self._stop_all()
        return CancelResponse.ACCEPT
    # workflow of this Trajectory arriver -> normalize waypoints -> start timer -> loop until trajectory finished -> figure out desired joint positions -> tell every joint where to go -> publish feedback -> repeat 
    def _execute_cb(self, goal_handle) -> FollowJointTrajectory.Result:
        #summary of first section - get the traj, validate, figure out timing, prepare feedback, start the timer
        goal = goal_handle.request # gets the trajectory request 
        #gets which joints to move
        joint_names = list(goal.trajectory.joint_names)
        waypoints = normalize_waypoints(goal.trajectory.points, len(joint_names)) #cleans and validates the trajectory points and turns them into usable Waypoint objects

        result = FollowJointTrajectory.Result() # this will get sendt back to the client 
        if not waypoints:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "No valid waypoints"
            return result # if there are no valid points, it rejects/aborts the movement and returns an error 

        duration = waypoints[-1].t # gets the total length of the trajectory, it uses the time of the last waypoint
        final_positions = list(waypoints[-1].positions) 
        control_dt = 1.0 / max(1.0, self.control_hz)
        fb_dt      = 1.0 / max(1.0, self.feedback_hz)
        next_fb    = 0.0

        self.get_logger().info( 
            f"[PI] Executing: joints={joint_names}, duration={duration:.2f}s" # prints what joints are being commanded and how long the trajectory is 
        )

        # ==========================================================
        # DEBUG TRACE 2: WHAT DID THE SERVER NORMALIZE THE GOAL TO?
        # ==========================================================
        if "swing_joint" in joint_names:
            si = joint_names.index("swing_joint")
            for i, wp in enumerate(waypoints):
                swing_rad = float(wp.positions[si])
                self.get_logger().warn(
                    f"[SWING DEBUG][NORMALIZED] point={i} t={wp.t:.3f}s "
                    f"target={swing_rad:+.6f} rad "
                    f"({math.degrees(swing_rad):+.2f} deg)"
                )

        # A new goal is an operator action, so clear any latched watchdog
        # fault from a previous run. Without this the first fault would
        # abort every subsequent goal instantly and the node would have to
        # be restarted to test anything.
        swing = self.joints.get("swing_joint")
        if swing is not None and hasattr(swing, "reset_swing_fault"):
            if getattr(swing, "_swing_fault", None):
                self.get_logger().warn(
                    f"[PI] clearing previous swing fault: {swing._swing_fault}"
                )
            swing.reset_swing_fault()
        #creates feedback messages and says which joints feedback will refer to  
        feedback = FollowJointTrajectory.Feedback() #
        feedback.joint_names = joint_names
        start = time.monotonic()
        
        while rclpy.ok(): # while ROS alive keep running, check cancel, check if done, calculate desired joint angles, tell each joint to move toward its target, wai 0.02 sec, repeat 
            elapsed = time.monotonic() - start # how far into the trajectory we are 

            if goal_handle.is_cancel_requested: # see if canceled and if so it gives back the message 
                self._stop_all()
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = "Canceled"
                return result

            if elapsed > duration:
                break

            desired = interpolate_positions(waypoints, elapsed)

            # ==========================================================
            # SWING - HOLD A FIXED TARGET INSTEAD OF CHASING
            # INTERPOLATED TARGETS
            # ==========================================================
            for i, jn in enumerate(joint_names):
                if jn == "swing_joint":

                    # Default to final swing target
                    swing_target = waypoints[-1].positions[i]

                     #Hold the next commanded waypoint as the target
                    for wp in waypoints:
                        if wp.t > elapsed + 1e-9:
                            swing_target = wp.positions[i]
                            break

                    desired[i] = swing_target

            # Phase 1: ask every joint what it wants to do. No hardware yet.
            actual_positions, errors = [], []
            plans: Dict[str, Tuple[int, int, float]] = {}

            for jn, tgt in zip(joint_names, desired):
                pos, err, _, direction, pwm = self.joints[jn].plan_toward_target(tgt)
                plans[jn] = (direction, pwm, err)
                actual_positions.append(pos)
                errors.append(err)

            # Phase 2: arbitrate the single shared PWM pin, then commit.
            self._apply_plans(plans)

            # Abort immediately on a latched swing watchdog fault.
            swing = self.joints.get("swing_joint")
            if swing is not None and getattr(swing, "_swing_fault", None):
                self.get_logger().error(f"[PI] SWING WATCHDOG: {swing._swing_fault}")
                self._stop_all()
                goal_handle.abort()
                return FollowJointTrajectory.Result()

            # building feedback messages 
            if elapsed >= next_fb:
                if swing is not None and "swing_joint" in plans:
                    d, p, e = plans["swing_joint"]
                    ooc = getattr(swing, "_swing_out_of_cal", None)
                    flag = f" OUT-OF-CAL(raw={ooc})" if ooc is not None else ""
                    requested_target = getattr(
                        swing, "_last_requested_target_rad", None
                    )
                    effective_target = getattr(
                        swing, "_last_effective_target_rad", None
                    )
                    current_pos = actual_positions[joint_names.index("swing_joint")]
                    raw_now = getattr(swing, "_last_valid_raw", None)

                    requested_deg = (
                        math.degrees(requested_target)
                        if requested_target is not None else float("nan")
                    )
                    effective_deg = (
                        math.degrees(effective_target)
                        if effective_target is not None else float("nan")
                    )

                    self.get_logger().warn(
                        f"[SWING DEBUG][CONTROL] t={elapsed:.2f}s "
                        f"requested={requested_deg:+.2f}deg "
                        f"effective={effective_deg:+.2f}deg "
                        f"current={math.degrees(current_pos):+.2f}deg "
                        f"error={math.degrees(e):+.2f}deg "
                        f"dir={d:+d} pwm={p} raw={raw_now}{flag}"
                    )
                dp = JointTrajectoryPoint()
                ap = JointTrajectoryPoint()
                ep = JointTrajectoryPoint()
                dp.positions = desired
                ap.positions = actual_positions
                ep.positions = errors
                dp.time_from_start = ap.time_from_start = ep.time_from_start = seconds_to_duration(elapsed)
                feedback.desired = dp
                feedback.actual  = ap
                feedback.error   = ep
                goal_handle.publish_feedback(feedback)
                next_fb += fb_dt

            time.sleep(control_dt)

        # Final settle
        final_target = waypoints[-1].positions # gets the final target positions from the last waypoint 
        settle_start = time.monotonic()
        while time.monotonic() - settle_start < 1.0: # gives robot one extra second to settle into the final phase 
            max_err = 0.0 # tracks error from all joints 
            plans = {}
            for jn, tgt in zip(joint_names, final_target): # commands each joint until its final target 
                _, err, _, direction, pwm = self.joints[jn].plan_toward_target(tgt)
                plans[jn] = (direction, pwm, err)
                max_err = max(max_err, abs(err)) # stores worst joint

            duty = self._apply_plans(plans)
            swing_raw = getattr(swing, "_last_valid_raw", None) if swing else None
            self.get_logger().info(
                f"[PI] duty={duty} max_err={max_err:.3f} swing_raw={swing_raw}"
            )

            if swing is not None and "swing_joint" in plans:
                d, p, e = plans["swing_joint"]
                requested_target = getattr(
                    swing, "_last_requested_target_rad", None
                )
                effective_target = getattr(
                    swing, "_last_effective_target_rad", None
                )
                current_pos = swing._last_position_rad

                requested_deg = (
                    math.degrees(requested_target)
                    if requested_target is not None else float("nan")
                )
                effective_deg = (
                    math.degrees(effective_target)
                    if effective_target is not None else float("nan")
                )

                self.get_logger().warn(
                    f"[SWING DEBUG][SETTLE] "
                    f"requested={requested_deg:+.2f}deg "
                    f"effective={effective_deg:+.2f}deg "
                    f"current={math.degrees(current_pos):+.2f}deg "
                    f"error={math.degrees(e):+.2f}deg "
                    f"dir={d:+d} pwm={p} raw={swing_raw} duty={duty}"
                )

            # The settle loop was reporting SUCCESSFUL even with a latched
            # fault: plan_toward_target() returns direction 0 once faulted,
            # so the motor stops, max_err never shrinks, the loop times out
            # after 1s and falls through to goal_handle.succeed(). Silent
            # wrong answer. Abort here too.
            if swing is not None and getattr(swing, "_swing_fault", None):
                self.get_logger().error(f"[PI] SWING WATCHDOG: {swing._swing_fault}")
                self._stop_all()
                goal_handle.abort()
                return FollowJointTrajectory.Result()

            if max_err <= self.goal_tolerance_rad:
                break
            time.sleep(control_dt)

        if self.stop_on_goal_finish: # stop the motors if trajectory finises
            self._stop_all()

        # Do not report success merely because the settle timer expired.
        # Success means the physical joints actually reached the final target
        # within the configured goal tolerance.
        if max_err > self.goal_tolerance_rad:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
            result.error_string = (
                "Final target not reached within settle timeout: "
                f"max error {max_err:.3f} rad exceeds "
                f"goal tolerance {self.goal_tolerance_rad:.3f} rad"
            )
            self.get_logger().error(f"[PI] {result.error_string}")
            return result

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "OK"
        self.get_logger().info("[PI] Trajectory finished")
        return result
        # marks the ROS action as successful 
    def _publish_joint_states(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        # read each joint 
        msg.name     = list(self.joints.keys())
        msg.position = [self.joints[n].read_position_rad() for n in msg.name]
        msg.velocity = [self.joints[n].read_velocity_rad_s() for n in msg.name]
        self.joint_state_pub.publish(msg)

    def _apply_plans(self, plans: Dict[str, Tuple[int, int, float]]) -> int:
        """
        Commit one control cycle across joints that share a single PWM pin.

        `plans` maps joint name -> (direction, pwm, err).
        Returns the duty cycle actually written, for logging.

        Every joint hangs off GPIO 7, so the hardware can only express one
        duty cycle. The old code wrote max() of every joint's request, which
        meant a joint asking for 145 to ease into its target got whatever the
        boom was asking for (up to 220) and overshot straight through the
        stop tolerance.
        """
        movers = {jn: p for jn, p in plans.items() if p[0] != 0}

        if not movers:
            for j in self.joints.values():
                j.stop()
            self.pi.set_PWM_dutycycle(self.shared_pwm_pin, 0)
            return 0

        if self.pwm_arbitration == "exclusive":
            # Serve the joint furthest from its target; hold the rest.
            winner = max(movers, key=lambda jn: abs(movers[jn][2]))
            for jn, j in self.joints.items():
                if jn == winner:
                    direction, pwm, _ = plans[jn]
                    j.apply_plan(direction, pwm)
                else:
                    j.stop()
            duty = plans[winner][1]
        else:
            # Legacy: everyone moves, everyone gets the highest duty asked for.
            for jn, j in self.joints.items():
                if jn in plans:
                    direction, pwm, _ = plans[jn]
                    j.apply_plan(direction, pwm)
                else:
                    j.stop()
            duty = max(p[1] for p in movers.values())

        self.pi.set_PWM_dutycycle(self.shared_pwm_pin, duty)
        return duty

    def _stop_all(self) -> None:
        # Turn off every motor direction
        for j in self.joints.values():
            j.stop()

        # Disable shared PWM
        self.pi.set_PWM_dutycycle(self.shared_pwm_pin, 0)

    def cleanup(self) -> None:
        self._stop_all()
        self.pi.write(self.stby_pin, 0)
        self.adc.close()
        self.pi.stop()


# ================================================================
# Shared sleep helper
# ================================================================

def _sleep_until(start: float, target_t: float) -> None:
    target = start + target_t
    while True:
        remaining = target - time.monotonic()
        if remaining <= 0:
            return
        time.sleep(min(remaining, 0.002))


# ================================================================
# Entry point
# ================================================================

def main() -> None:
    parser = argparse.ArgumentParser(description="Unified excavator trajectory server")
    parser.add_argument(
        "--mode",
        choices=["pi", "sim"],
        default=None,
        help=(
            "Force 'pi' (Raspberry Pi real-robot) or 'sim' (Isaac Sim / ROS computer). "
            "If omitted, auto-detected via pigpio availability."
        ),
    )
    parser.add_argument(
        "--config",
        default=None,
        help=(
            "Excavator YAML configuration. Defaults to the installed "
            "config/excavator1.yaml."
        ),
    )

    # parse only known args so ROS 2 launch args don't cause errors
    args, _ = parser.parse_known_args()

    mode = _detect_mode(args.mode)
    print(f"[excavator_trajectory_server] Detected mode: {mode.upper()}", flush=True) # lets you choose between ros or issac sim 

    if mode == "pi": # load pi hardware librarier, starts ros, create the real excavtor controller, keep it running 
        _import_pi_libs()
        config_path = args.config or _default_config_path()
        print(
            f"[excavator_trajectory_server] Config: {config_path}",
            flush=True,
        )
        rclpy.init()
        node = PiExcavatorTrajectoryServer(config_path=config_path)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            try:
                node.cleanup()
                node.destroy_node()
            except Exception:
                pass
            if rclpy.ok():
                rclpy.shutdown()
    else: # isaac sim
        rclpy.init()
        node = SimTrajectoryServer()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            try:
                node.destroy_node()
            except Exception:
                pass
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()