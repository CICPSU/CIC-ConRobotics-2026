#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scoop_trajectory_client.py

Trajectory client that reads an external "spec" file.

Spec types:
1) type: joint
   - joint-space keyframes (recommended: radians)
2) type: task
   - task-space waypoints for end-effector (bucket_end_link)
   - converted to joint-space using an IK solver (default: planar_approx)

v3 extension (for task specs):
- Optional swing profile can be provided and will be merged into the IK result:
    swing:
      joint: swing_joint
      unit: rad
      keyframes:
        - {t: 0.0, q: 0.0}
        - {t: 8.0, q: 1.2}

This keeps IK responsible for boom/arm/bucket while swing is handled as a
separate, time-parameterized joint profile (still driven from the same task spec).

Usage:
  python3 scoop_trajectory_client.py --spec specs/scoop_cycle_v1.yaml
  python3 scoop_trajectory_client.py --spec specs/scoop_cycle_task_v3.yaml
"""

from __future__ import annotations

import argparse
import json
import math
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from robot_model import RobotModel
from ik.ik_interface import TaskPose
from ik.ik_planar_approx import PlanarApproxIK, PlanarChainConfig


def seconds_to_duration(t: float) -> Duration:
    d = Duration()
    d.sec = int(t)
    d.nanosec = int((t - d.sec) * 1e9)
    return d


def densify_linear(
    waypoints: List[Tuple[float, List[float]]],
    max_dt: float,
) -> List[Tuple[float, List[float]]]:
    if len(waypoints) < 2:
        return waypoints

    out: List[Tuple[float, List[float]]] = [waypoints[0]]
    for (t0, p0), (t1, p1) in zip(waypoints[:-1], waypoints[1:]):
        dt = t1 - t0
        if dt <= 0.0:
            continue
        steps = max(0, int(math.floor(dt / max_dt)) - 1)
        for k in range(1, steps + 1):
            u = k / (steps + 1)
            tk = t0 + u * dt
            pk = [a + u * (b - a) for a, b in zip(p0, p1)]
            out.append((tk, pk))
        out.append((t1, p1))

    # enforce strictly increasing time
    eps = 1e-4
    fixed: List[Tuple[float, List[float]]] = []
    last_t = -1e9
    for t, p in out:
        if t <= last_t:
            t = last_t + eps
        fixed.append((t, p))
        last_t = t
    return fixed


def load_spec(path: str) -> Dict[str, Any]:
    """Load YAML if available, else JSON."""
    if path.lower().endswith(".json"):
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)

    # Try YAML
    try:
        import yaml  # type: ignore
    except Exception as e:
        raise RuntimeError(
            "YAML spec requested but PyYAML is not available. "
            "Install it (pip install pyyaml) or use a .json spec."
        ) from e

    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError("Spec file must be a mapping (dict) at the top level.")
    return data


def unit_to_radians(unit: str, value: float) -> float:
    u = unit.strip().lower()
    if u in ("rad", "radian", "radians"):
        return float(value)
    if u in ("deg", "degree", "degrees"):
        return math.radians(float(value))
    raise ValueError(f"Unknown angle unit: {unit}")


def interp_profile(keyframes: List[Tuple[float, float]], t: float) -> float:
    """
    Linear interpolation over (t, q) keyframes.
    If t is outside the range, clamp to endpoints.
    """
    if not keyframes:
        return 0.0
    if t <= keyframes[0][0]:
        return keyframes[0][1]
    if t >= keyframes[-1][0]:
        return keyframes[-1][1]

    # Find segment
    for (t0, q0), (t1, q1) in zip(keyframes[:-1], keyframes[1:]):
        if t0 <= t <= t1:
            if t1 <= t0:
                return q1
            u = (t - t0) / (t1 - t0)
            return q0 + u * (q1 - q0)
    return keyframes[-1][1]


class ScoopTrajectoryClient(Node):
    def __init__(self, action_name: str) -> None:
        super().__init__("scoop_trajectory_client")
        self._action_name = action_name
        self._client = ActionClient(self, FollowJointTrajectory, self._action_name)

    def send_joint_trajectory(
        self,
        joints: List[str],
        waypoints: List[Tuple[float, List[float]]],
        wait_sec: float = 5.0
    ) -> int:
        if not self._client.wait_for_server(timeout_sec=wait_sec):
            self.get_logger().error(f"Action server not available: {self._action_name}")
            return 1

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joints
        goal.trajectory.points = []

        for t, pos in waypoints:
            pt = JointTrajectoryPoint()
            pt.positions = pos
            pt.time_from_start = seconds_to_duration(t)
            goal.trajectory.points.append(pt)

        self.get_logger().info(
            f"Sending: joints={joints}, points={len(goal.trajectory.points)}, duration={waypoints[-1][0]:.2f}s"
        )

        send_future = self._client.send_goal_async(goal, feedback_callback=self.feedback_cb)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return 2

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        self.get_logger().info(
            f"Result: error_code={result.error_code}, error_string='{result.error_string}'"
        )
        return 0

    def feedback_cb(self, fb_msg) -> None:
        pass


def build_from_joint_spec(spec: Dict[str, Any]) -> Tuple[List[str], List[Tuple[float, List[float]]]]:
    unit = str(spec.get("unit", "rad"))
    joints = list(spec["joints"])
    keyframes = list(spec["keyframes"])
    interp = str(spec.get("interp", "linear"))
    if interp != "linear":
        raise ValueError("Only linear interpolation is supported for now.")

    waypoints: List[Tuple[float, List[float]]] = []
    for kf in keyframes:
        t = float(kf["t"])
        q_in = list(kf["q"])
        if len(q_in) != len(joints):
            raise ValueError("Keyframe q length must match joints length.")
        q = [unit_to_radians(unit, float(v)) for v in q_in]
        waypoints.append((t, q))

    waypoints.sort(key=lambda x: x[0])
    max_dt = float(spec.get("densify_dt", 0.2))
    return joints, densify_linear(waypoints, max_dt=max_dt)


def build_from_task_spec(spec: Dict[str, Any], model: RobotModel) -> Tuple[List[str], List[Tuple[float, List[float]]]]:
    frame = str(spec.get("frame", "base_link"))
    end_effector = str(spec.get("end_effector", "bucket_end_link"))
    if frame != "base_link":
        raise ValueError("This implementation assumes frame=base_link.")

    ik_cfg = spec.get("ik", {}) or {}
    solver_name = str(ik_cfg.get("solver", "planar_approx"))
    if solver_name != "planar_approx":
        raise ValueError("Only solver=planar_approx is supported.")

    chain_cfg = PlanarChainConfig(base_link="base_link", tip_link=end_effector)
    solver = PlanarApproxIK(model=model, cfg=chain_cfg)

    wps_in = list(spec["waypoints"])
    wps_in.sort(key=lambda w: float(w["t"]))

    seed_mode = str(ik_cfg.get("seed_mode", "previous")).lower()
    seed: Optional[Dict[str, float]] = ik_cfg.get("seed", None) or None
    last_solution: Optional[Dict[str, float]] = None

    base_joint_names: Optional[List[str]] = None
    base_waypoints: List[Tuple[float, List[float]]] = []

    for wp in wps_in:
        t = float(wp["t"])
        pose_d = wp["pose"]
        pose = TaskPose(
            x=float(pose_d["x"]),
            y=float(pose_d.get("y", 0.0)),
            z=float(pose_d["z"]),
            pitch=float(pose_d.get("pitch", 0.0)),
        )

        if seed_mode == "previous" and last_solution is not None:
            seed = last_solution

        res = solver.solve(pose=pose, seed=seed)
        if not res.success:
            raise RuntimeError(f"IK failed at t={t:.2f}: {res.reason}")

        if base_joint_names is None:
            base_joint_names = list(res.joint_names)

        base_waypoints.append((t, list(res.positions)))
        last_solution = dict(zip(res.joint_names, res.positions))

    if base_joint_names is None:
        raise ValueError("No waypoints produced from task spec.")

    # Optional: merge swing profile into the trajectory.
    joints_out = list(base_joint_names)
    merged_waypoints: List[Tuple[float, List[float]]] = list(base_waypoints)

    swing_spec = spec.get("swing", None)
    if swing_spec:
        swing_joint = str(swing_spec.get("joint", "swing_joint"))
        swing_unit = str(swing_spec.get("unit", "rad"))
        swing_kfs_raw = list(swing_spec.get("keyframes", []))
        if not swing_kfs_raw:
            raise ValueError("swing.keyframes must be a non-empty list when swing is provided.")

        swing_kfs: List[Tuple[float, float]] = []
        for kf in swing_kfs_raw:
            tk = float(kf["t"])
            qk = unit_to_radians(swing_unit, float(kf["q"]))
            swing_kfs.append((tk, qk))
        swing_kfs.sort(key=lambda x: x[0])

        joints_out = [swing_joint] + joints_out
        merged_waypoints = []
        for t, q_base in base_waypoints:
            q_swing = interp_profile(swing_kfs, t)
            merged_waypoints.append((t, [q_swing] + q_base))

    # Optional: append constant joints (e.g., bucket_end_joint)
    append_joints = spec.get("append_joints", None)
    if append_joints:
        # Preserve insertion order if YAML preserves it; otherwise, sort keys for stability.
        items = list(append_joints.items())
        for jname, jval in items:
            joints_out.append(str(jname))
            for i, (t, q) in enumerate(merged_waypoints):
                merged_waypoints[i] = (t, q + [unit_to_radians("rad", float(jval))])

    max_dt = float(spec.get("densify_dt", 0.1))
    return joints_out, densify_linear(merged_waypoints, max_dt=max_dt)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--spec", required=True, help="Path to spec file (.yaml or .json)")
    parser.add_argument("--action", default="/upper_arm_controller/follow_joint_trajectory", help="Action name")
    parser.add_argument("--urdf", default="robot/zx200.urdf", help="URDF path (used by IK task specs)")
    args = parser.parse_args()

    spec = load_spec(args.spec)
    spec_type = str(spec.get("type", "")).strip().lower()
    if spec_type not in ("joint", "task"):
        raise ValueError("Spec must define type: joint or task")

    model = RobotModel.from_urdf_file(args.urdf)

    if spec_type == "joint":
        joints, wps = build_from_joint_spec(spec)
    else:
        joints, wps = build_from_task_spec(spec, model=model)

    rclpy.init()
    node = ScoopTrajectoryClient(action_name=args.action)
    try:
        rc = node.send_joint_trajectory(joints, wps)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(rc)


if __name__ == "__main__":
    main()

