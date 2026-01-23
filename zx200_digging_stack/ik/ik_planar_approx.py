# -*- coding: utf-8 -*-
"""
ik_planar_approx.py

Planar (X-Z) approximation IK for an excavator-like 3DOF arm.

Client compatibility:
  from ik.ik_planar_approx import PlanarApproxIK, PlanarChainConfig
  chain_cfg = PlanarChainConfig(base_link="base_link", tip_link="bucket_end_link")
  solver = PlanarApproxIK(model=model, cfg=chain_cfg)
  res = solver.solve(pose=pose, seed=seed)

The client code expects IKResult to have:
  - res.joint_names
  - res.positions
Optionally, we also provide:
  - res.q (dict)

Robustness:
- URDF joint origins often do not encode physical link lengths.
  If inferred lengths are too small, fallback lengths are used.
- If target is out of reach, the wrist point is projected onto the reachable annulus
  rather than failing hard.

All angles are radians.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
import math


@dataclass
class PlanarChainConfig:
    """
    The client expects this class name and these keyword args:
      PlanarChainConfig(base_link="...", tip_link="...")
    """
    base_link: str = "base_link"
    tip_link: str = "bucket_end_link"

    # Joint names (match URDF)
    boom_joint: str = "boom_joint"
    arm_joint: str = "arm_joint"
    bucket_joint: str = "bucket_joint"

    # Optional joints (not solved here; kept for completeness)
    swing_joint: str = "swing_joint"
    bucket_end_joint: str = "bucket_end_joint"

    # Fallback segment lengths in meters (used when URDF origins do not encode geometry)
    fallback_L1: float = 1.00  # boom
    fallback_L2: float = 1.00  # arm
    fallback_L3: float = 0.50  # bucket segment (wrist->tip)

    # If inferred length < min_len, treat it as unreliable and use fallback
    min_len: float = 0.05

    # Elbow configuration: +1 elbow-down, -1 elbow-up
    elbow_sign: int = +1

    # Clamp final joint angles to URDF limits if present
    clamp_to_limits: bool = True


# Backward-compatible alias (some code may import IKConfig)
IKConfig = PlanarChainConfig


@dataclass
class IKResult:
    """
    Compatible result container.

    Many clients expect:
      - joint_names: List[str]
      - positions:   List[float]
    We also include q for convenience.
    """
    success: bool
    joint_names: List[str]
    positions: List[float]
    q: Dict[str, float]
    reason: str = ""


class PlanarApproxIK:
    """
    3-DOF planar IK in X-Z plane.

    Convention:
      - X forward, Z up
      - pitch=0 points along +X; positive pitch rotates toward +Z
    """

    def __init__(
        self,
        model,
        cfg: Optional[PlanarChainConfig] = None,
        chain_cfg: Optional[PlanarChainConfig] = None,
        ee_link: Optional[str] = None,
    ):
        """
        Compatibility notes:
          - The client calls: PlanarApproxIK(model=model, cfg=chain_cfg)
          - Some variants may call: PlanarApproxIK(model=model, chain_cfg=...)
          - We accept both cfg= and chain_cfg=.

        model must provide:
          - get_joint(name) returning an object with:
              .origin_xyz (tuple3), .has_limit (bool), .limit_lower, .limit_upper, .type
        """
        self.model = model
        self.cfg: PlanarChainConfig = cfg or chain_cfg or PlanarChainConfig()

        # Optional override for tip link name (not used in math here, but kept for consistency)
        if ee_link is not None:
            self.cfg.tip_link = ee_link

        self.chain: List[str] = [self.cfg.boom_joint, self.cfg.arm_joint, self.cfg.bucket_joint]
        self.L1, self.L2, self.L3 = self._infer_lengths()

    def solve(
        self,
        pose=None,
        target_xyz: Optional[Tuple[float, float, float]] = None,
        target_pitch: Optional[float] = None,
        seed: Optional[Dict[str, float]] = None,
    ) -> IKResult:
        """
        Solve IK using either:
          - solve(pose=pose_dict_or_obj, seed=...)
          - solve(target_xyz=(x,y,z), target_pitch=pitch, seed=...)

        pose is expected to provide:
          - x, y, z
          - pitch (radians)

        Returns IKResult with:
          - joint_names
          - positions
          - q
        """
        # --- Unify inputs ---
        if pose is not None:
            if isinstance(pose, dict):
                x = float(pose.get("x", 0.0))
                y = float(pose.get("y", 0.0))
                z = float(pose.get("z", 0.0))
                pitch = float(pose.get("pitch", 0.0))
            else:
                x = float(getattr(pose, "x"))
                y = float(getattr(pose, "y"))
                z = float(getattr(pose, "z"))
                pitch = float(getattr(pose, "pitch"))
        else:
            if target_xyz is None or target_pitch is None:
                return IKResult(
                    success=False,
                    joint_names=[],
                    positions=[],
                    q={},
                    reason="No pose or target specified",
                )
            x, y, z = target_xyz
            pitch = float(target_pitch)

        # --- Planar solve in X-Z ---
        L1, L2, L3 = self.L1, self.L2, self.L3

        # Wrist position (back off by bucket segment along pitch)
        wx = x - L3 * math.cos(pitch)
        wz = z - L3 * math.sin(pitch)

        r2 = wx * wx + wz * wz
        r = math.sqrt(r2)

        # Reachable annulus
        Rmax = (L1 + L2) - 1e-6
        Rmin = abs(L1 - L2) + 1e-6

        # Project wrist onto reachable region rather than failing
        if r > Rmax or r < Rmin:
            if r < 1e-9:
                wx, wz = Rmin, 0.0
            else:
                r_clamped = min(max(r, Rmin), Rmax)
                s = r_clamped / r
                wx *= s
                wz *= s
            r2 = wx * wx + wz * wz

        # Elbow (law of cosines)
        c2 = (r2 - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
        c2 = max(-1.0, min(1.0, c2))

        s2 = math.sqrt(max(0.0, 1.0 - c2 * c2))
        if self.cfg.elbow_sign < 0:
            s2 = -s2

        q2 = math.atan2(s2, c2)  # arm joint

        # Shoulder
        k1 = L1 + L2 * c2
        k2 = L2 * s2
        q1 = math.atan2(wz, wx) - math.atan2(k2, k1)  # boom joint

        # Bucket pitch closure: pitch ≈ q1 + q2 + q3
        q3 = pitch - (q1 + q2)

        # Keep a stable joint order (client-friendly)
        joint_names = [self.cfg.boom_joint, self.cfg.arm_joint, self.cfg.bucket_joint]
        q = {
            self.cfg.boom_joint: q1,
            self.cfg.arm_joint: q2,
            self.cfg.bucket_joint: q3,
        }

        if self.cfg.clamp_to_limits:
            q = self._clamp_dict_to_limits(q)

        positions = [float(q[j]) for j in joint_names]

        return IKResult(
            success=True,
            joint_names=joint_names,
            positions=positions,
            q=q,
            reason="OK",
        )

    def _infer_lengths(self) -> Tuple[float, float, float]:
        """
        Infer approximate segment lengths (L1,L2,L3) from URDF joint origins (X-Z components).
        If origins are not meaningful (very small), fallback lengths are used.
        """
        fallback = {
            self.cfg.boom_joint: self.cfg.fallback_L1,
            self.cfg.arm_joint: self.cfg.fallback_L2,
            self.cfg.bucket_joint: self.cfg.fallback_L3,
        }

        lengths_by_joint: Dict[str, float] = {}

        for jn in self.chain:
            try:
                ji = self.model.get_joint(jn)
            except Exception:
                continue

            dx, _, dz = getattr(ji, "origin_xyz", (0.0, 0.0, 0.0))
            L = math.hypot(float(dx), float(dz))

            if L >= self.cfg.min_len:
                lengths_by_joint[jn] = L

        L1 = lengths_by_joint.get(self.cfg.boom_joint, fallback[self.cfg.boom_joint])
        L2 = lengths_by_joint.get(self.cfg.arm_joint, fallback[self.cfg.arm_joint])
        L3 = lengths_by_joint.get(self.cfg.bucket_joint, fallback[self.cfg.bucket_joint])

        # Final sanity clamp (avoid pathological tiny lengths)
        L1 = max(L1, fallback[self.cfg.boom_joint])
        L2 = max(L2, fallback[self.cfg.arm_joint])
        L3 = max(L3, fallback[self.cfg.bucket_joint])

        return (L1, L2, L3)

    def _clamp_dict_to_limits(self, q: Dict[str, float]) -> Dict[str, float]:
        """
        Clamp joint angles to URDF limits when available.
        Continuous joints are not clamped.
        """
        out = dict(q)

        for jn, val in q.items():
            try:
                ji = self.model.get_joint(jn)
            except Exception:
                continue

            jtype = str(getattr(ji, "type", "")).lower()
            if jtype == "continuous":
                continue

            has_limit = bool(getattr(ji, "has_limit", False))
            if has_limit:
                lo = float(getattr(ji, "limit_lower", -1e9))
                hi = float(getattr(ji, "limit_upper", 1e9))
                if lo > hi:
                    lo, hi = hi, lo
                out[jn] = max(lo, min(hi, float(val)))

        return out
