#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scoop_trajectory_server.py

TrajectoryBridgeServer:
- Provides FollowJointTrajectory action server
- Publishes sensor_msgs/JointState to /joint_command for Isaac Sim

Defaults are chosen to minimize "typing flags" in terminal:
- use_degrees=False (ROS standard is radians; Isaac expects radians)
- reliable=True (to satisfy Isaac Sim nodes that request RELIABLE)
- enforce_limits=True (clamp using URDF limits as a safety net)

URDF parsing is done via robot_model.py (shared module).
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple, Optional

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from control_msgs.action import FollowJointTrajectory
from rclpy.action import CancelResponse, GoalResponse
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from robot_model import RobotModel


def duration_to_seconds(d) -> float:
    return float(d.sec) + float(d.nanosec) * 1e-9


def seconds_to_duration(t: float):
    sec = int(math.floor(t))
    nanosec = int((t - sec) * 1e9)
    from builtin_interfaces.msg import Duration
    return Duration(sec=sec, nanosec=nanosec)


@dataclass
class Waypoint:
    t: float
    positions: List[float]


class TrajectoryBridgeServer(Node):
    def __init__(self) -> None:
        super().__init__("scoop_trajectory_server")

        # Core params
        self.declare_parameter("action_name", "/upper_arm_controller/follow_joint_trajectory")
        self.declare_parameter("command_topic", "/joint_command")
        self.declare_parameter("publish_hz", 60.0)
        self.declare_parameter("hold_last_seconds", 0.30)
        self.declare_parameter("feedback_hz", 10.0)

        # Behavior defaults (avoid terminal flags)
        self.declare_parameter("use_degrees", False)       # incoming trajectory units
        self.declare_parameter("enforce_limits", True)     # clamp using URDF limits
        self.declare_parameter("reliable", True)           # publisher QoS reliability
        self.declare_parameter("qos_depth", 10)

        # URDF path (shared for future updates)
        self.declare_parameter("urdf_path", "robot/zx200.urdf")

        self._action_name: str = self.get_parameter("action_name").value
        self._command_topic: str = self.get_parameter("command_topic").value
        self._publish_hz: float = float(self.get_parameter("publish_hz").value)
        self._hold_last_seconds: float = float(self.get_parameter("hold_last_seconds").value)
        self._feedback_hz: float = float(self.get_parameter("feedback_hz").value)

        self._use_degrees: bool = bool(self.get_parameter("use_degrees").value)
        self._enforce_limits: bool = bool(self.get_parameter("enforce_limits").value)
        self._reliable: bool = bool(self.get_parameter("reliable").value)
        self._qos_depth: int = int(self.get_parameter("qos_depth").value)
        self._urdf_path: str = str(self.get_parameter("urdf_path").value)

        if self._publish_hz <= 0.0:
            self._publish_hz = 60.0
        if self._feedback_hz <= 0.0:
            self._feedback_hz = 10.0
        if self._qos_depth <= 0:
            self._qos_depth = 10

        # Load URDF model and build clamp limits in radians
        self._model = RobotModel.from_urdf_file(self._urdf_path)
        self._limits_rad: Dict[str, Tuple[float, float]] = {}
        for jn in self._model.joint_names():
            ji = self._model.get_joint(jn)
            if ji.joint_type == "continuous":
                continue
            lim = ji.limit
            if lim.lower is None or lim.upper is None:
                continue
            self._limits_rad[jn] = (float(lim.lower), float(lim.upper))

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=self._qos_depth,
            reliability=ReliabilityPolicy.RELIABLE if self._reliable else ReliabilityPolicy.BEST_EFFORT,
        )
        self._pub = self.create_publisher(JointState, self._command_topic, qos)

        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            self._action_name,
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
        )

        self.get_logger().info("TrajectoryBridgeServer ready")
        self.get_logger().info(f"Action:    {self._action_name}")
        self.get_logger().info(f"Publishes: {self._command_topic} (sensor_msgs/JointState)")
        self.get_logger().info(
            f"publish_hz={self._publish_hz:.1f}, hold_last_seconds={self._hold_last_seconds:.2f}, "
            f"feedback_hz={self._feedback_hz:.1f}"
        )
        self.get_logger().info(
            f"use_degrees={self._use_degrees}, enforce_limits={self._enforce_limits}, "
            f"reliable={self._reliable}, qos_depth={self._qos_depth}"
        )
        self.get_logger().info(f"URDF: {self._urdf_path} (limits loaded: {len(self._limits_rad)})")

    def goal_cb(self, goal_request: FollowJointTrajectory.Goal) -> int:
        if not goal_request.trajectory.joint_names:
            self.get_logger().warn("Rejected goal: empty joint_names")
            return GoalResponse.REJECT

        if not goal_request.trajectory.points:
            self.get_logger().warn("Rejected goal: empty points")
            return GoalResponse.REJECT

        n = len(goal_request.trajectory.joint_names)
        for i, p in enumerate(goal_request.trajectory.points):
            if len(p.positions) not in (0, n):
                self.get_logger().warn(
                    f"Rejected goal: point[{i}] positions size {len(p.positions)} != {n}"
                )
                return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle) -> int:
        self.get_logger().info("Cancel request received")
        return CancelResponse.ACCEPT

    def execute_cb(self, goal_handle) -> FollowJointTrajectory.Result:
        goal: FollowJointTrajectory.Goal = goal_handle.request
        joint_names: List[str] = list(goal.trajectory.joint_names)

        waypoints = self._normalize_waypoints(goal.trajectory.points, len(joint_names))
        if len(waypoints) == 0:
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "No valid waypoints"
            return result

        duration = waypoints[-1].t
        dt = 1.0 / self._publish_hz
        fb_dt = 1.0 / self._feedback_hz

        self.get_logger().info(
            f"Executing: joints={joint_names}, points={len(waypoints)}, duration={duration:.2f}s"
        )

        start = time.monotonic()
        next_feedback_time = 0.0

        js = JointState()
        js.name = joint_names

        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = joint_names

        last_positions = waypoints[0].positions

        t = 0.0
        while t <= duration + 1e-9:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().warn("Trajectory canceled")
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = "Canceled"
                return result

            positions = self._interp_positions(waypoints, t)
            positions = self._to_radians_and_clamp(joint_names, positions)
            last_positions = positions

            js.header.stamp = self.get_clock().now().to_msg()
            js.position = positions
            self._pub.publish(js)

            if t >= next_feedback_time:
                desired = JointTrajectoryPoint()
                desired.positions = positions
                desired.time_from_start = seconds_to_duration(t)
                feedback.desired = desired
                goal_handle.publish_feedback(feedback)
                next_feedback_time += fb_dt

            self._sleep_until(start, t + dt)
            t = (time.monotonic() - start)

        if self._hold_last_seconds > 0.0:
            hold_end = time.monotonic() + self._hold_last_seconds
            while time.monotonic() < hold_end:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    break
                js.header.stamp = self.get_clock().now().to_msg()
                js.position = last_positions
                self._pub.publish(js)
                time.sleep(dt)

        goal_handle.succeed()
        self.get_logger().info("Trajectory done")

        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "OK"
        return result

    def _to_radians_and_clamp(self, joint_names: List[str], positions: List[float]) -> List[float]:
        out: List[float] = []
        for jn, v_in in zip(joint_names, positions):
            v = float(v_in)
            if self._use_degrees:
                v = math.radians(v)

            if self._enforce_limits and jn in self._limits_rad:
                lo, hi = self._limits_rad[jn]
                v = max(lo, min(hi, v))

            out.append(v)
        return out

    def _normalize_waypoints(self, points: Sequence[JointTrajectoryPoint], n_joints: int) -> List[Waypoint]:
        out: List[Waypoint] = []
        last_t = -1e9

        for p in points:
            t = duration_to_seconds(p.time_from_start)
            if t < 0.0:
                t = 0.0
            if t + 1e-9 < last_t:
                self.get_logger().warn("Trajectory points are not time-ordered; rejecting goal")
                return []
            last_t = t

            if len(p.positions) == 0:
                self.get_logger().warn("A trajectory point has empty positions; rejecting goal")
                return []
            if len(p.positions) != n_joints:
                self.get_logger().warn("A trajectory point has wrong positions length; rejecting goal")
                return []

            out.append(Waypoint(t=t, positions=list(p.positions)))

        if out and out[0].t > 1e-9:
            out.insert(0, Waypoint(t=0.0, positions=list(out[0].positions)))

        if len(out) >= 2 and abs(out[-1].t - out[0].t) < 1e-9:
            out[-1].t = out[0].t + 1e-3

        return out

    def _interp_positions(self, waypoints: Sequence[Waypoint], t: float) -> List[float]:
        if t <= waypoints[0].t:
            return list(waypoints[0].positions)
        if t >= waypoints[-1].t:
            return list(waypoints[-1].positions)

        i = 0
        while i + 1 < len(waypoints) and waypoints[i + 1].t < t:
            i += 1

        w0 = waypoints[i]
        w1 = waypoints[i + 1]

        if abs(w1.t - w0.t) < 1e-12:
            return list(w1.positions)

        alpha = (t - w0.t) / (w1.t - w0.t)
        return [p0 + alpha * (p1 - p0) for p0, p1 in zip(w0.positions, w1.positions)]

    @staticmethod
    def _sleep_until(start: float, target_t: float) -> None:
        target = start + target_t
        while True:
            now = time.monotonic()
            remaining = target - now
            if remaining <= 0:
                return
            time.sleep(min(remaining, 0.002))


def main() -> None:
    rclpy.init()
    node = TrajectoryBridgeServer()
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
