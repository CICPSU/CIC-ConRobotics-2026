# -*- coding: utf-8 -*-
"""
ik_interface.py

Defines a minimal IK interface used by the trajectory client.

We keep this lightweight so you can later swap:
- planar_approx (geometric)
- MoveIt IK
- IKFast plugin
without changing the client code.
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, List, Optional, Protocol, Tuple


@dataclass(frozen=True)
class TaskPose:
    """
    Simple task pose for excavator digging in a world-fixed frame.

    Convention:
    - x, y, z in meters (base_link frame by default)
    - pitch in radians (rotation about +Y for X-Z plane motion)
      pitch=0 means end-effector points along +X in the X-Z plane.
    """
    x: float
    y: float
    z: float
    pitch: float


@dataclass(frozen=True)
class IKResult:
    joint_names: List[str]
    positions: List[float]
    success: bool
    reason: str = ""


class IKSolver(Protocol):
    def solve(self, pose: TaskPose, seed: Optional[Dict[str, float]] = None) -> IKResult:
        ...
