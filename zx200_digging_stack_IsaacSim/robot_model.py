#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
robot_model.py

Small, dependency-light URDF parser for:
- Joint list (name/type/parent/child/axis/origin)
- Joint limits (lower/upper/effort/velocity)
- Chain extraction (base_link -> end_effector_link)
- Approximate planar link lengths from joint origins (X-Z plane)

This module is intentionally shared by:
- The trajectory bridge server (to clamp to URDF limits safely)
- The IK layer (to build kinematic chains that survive URDF updates)

Notes:
- URDF joints specify an <origin xyz="" rpy=""> relative transform between parent and child.
- For excavator arm kinematics we mostly care about the joint sequence and the origin offsets.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import math
import xml.etree.ElementTree as ET


@dataclass(frozen=True)
class JointLimit:
    lower: Optional[float] = None
    upper: Optional[float] = None
    effort: Optional[float] = None
    velocity: Optional[float] = None


@dataclass(frozen=True)
class JointInfo:
    name: str
    joint_type: str
    parent: str
    child: str
    axis: Tuple[float, float, float]
    origin_xyz: Tuple[float, float, float]
    origin_rpy: Tuple[float, float, float]
    limit: JointLimit


def _parse_floats(s: Optional[str], n: int, default: float = 0.0) -> Tuple[float, ...]:
    if not s:
        return tuple([default] * n)
    parts = s.replace(",", " ").split()
    vals = []
    for i in range(n):
        if i < len(parts):
            try:
                vals.append(float(parts[i]))
            except ValueError:
                vals.append(default)
        else:
            vals.append(default)
    return tuple(vals)


def _safe_float(s: Optional[str]) -> Optional[float]:
    if s is None:
        return None
    try:
        return float(s)
    except ValueError:
        return None


class RobotModel:
    """
    Parsed URDF model representation.
    """

    def __init__(self, joints: Dict[str, JointInfo]) -> None:
        self.joints: Dict[str, JointInfo] = joints
        self._parent_to_joints: Dict[str, List[str]] = {}
        self._child_to_joint: Dict[str, str] = {}

        for jn, ji in joints.items():
            self._parent_to_joints.setdefault(ji.parent, []).append(jn)
            self._child_to_joint[ji.child] = jn

    @staticmethod
    def from_urdf_file(path: str) -> "RobotModel":
        tree = ET.parse(path)
        root = tree.getroot()

        joints: Dict[str, JointInfo] = {}

        for j in root.findall("joint"):
            name = j.get("name", "")
            joint_type = j.get("type", "")

            parent_el = j.find("parent")
            child_el = j.find("child")
            if parent_el is None or child_el is None:
                continue
            parent = parent_el.get("link", "")
            child = child_el.get("link", "")

            axis_el = j.find("axis")
            axis = (1.0, 0.0, 0.0)
            if axis_el is not None:
                axis = _parse_floats(axis_el.get("xyz"), 3, 0.0)  # type: ignore

            origin_el = j.find("origin")
            origin_xyz = (0.0, 0.0, 0.0)
            origin_rpy = (0.0, 0.0, 0.0)
            if origin_el is not None:
                origin_xyz = _parse_floats(origin_el.get("xyz"), 3, 0.0)  # type: ignore
                origin_rpy = _parse_floats(origin_el.get("rpy"), 3, 0.0)  # type: ignore

            limit_el = j.find("limit")
            limit = JointLimit()
            if limit_el is not None:
                limit = JointLimit(
                    lower=_safe_float(limit_el.get("lower")),
                    upper=_safe_float(limit_el.get("upper")),
                    effort=_safe_float(limit_el.get("effort")),
                    velocity=_safe_float(limit_el.get("velocity")),
                )

            joints[name] = JointInfo(
                name=name,
                joint_type=joint_type,
                parent=parent,
                child=child,
                axis=(float(axis[0]), float(axis[1]), float(axis[2])),
                origin_xyz=(float(origin_xyz[0]), float(origin_xyz[1]), float(origin_xyz[2])),
                origin_rpy=(float(origin_rpy[0]), float(origin_rpy[1]), float(origin_rpy[2])),
                limit=limit,
            )

        return RobotModel(joints=joints)

    def joint_names(self) -> List[str]:
        return list(self.joints.keys())

    def get_joint(self, name: str) -> JointInfo:
        return self.joints[name]

    def joint_limit(self, name: str) -> JointLimit:
        return self.joints[name].limit

    def chain_joints(self, base_link: str, tip_link: str) -> List[str]:
        """
        Returns joint names from base_link to tip_link (exclusive of base_link, inclusive of the joint that creates tip_link).
        We walk tip_link -> base_link using child->joint mapping.
        """
        chain_rev: List[str] = []
        cur = tip_link
        visited = set()
        while cur != base_link:
            if cur in visited:
                raise RuntimeError(f"Cycle detected while building chain: {cur}")
            visited.add(cur)

            if cur not in self._child_to_joint:
                raise KeyError(f"No joint produces child link '{cur}'. Cannot build chain from '{base_link}' to '{tip_link}'.")
            jn = self._child_to_joint[cur]
            chain_rev.append(jn)
            cur = self.joints[jn].parent

        return list(reversed(chain_rev))

    def chain_links(self, base_link: str, tip_link: str) -> List[str]:
        joints = self.chain_joints(base_link, tip_link)
        links = [base_link]
        cur = base_link
        for jn in joints:
            ji = self.joints[jn]
            if ji.parent != cur:
                # This can happen if there are fixed joints etc; still produce consistent output
                cur = ji.parent
            cur = ji.child
            links.append(cur)
        return links

    def planar_link_lengths_xz(self, base_link: str, tip_link: str, joints_override: Optional[List[str]] = None) -> List[float]:
        """
        Approximate planar link lengths in X-Z plane using joint origin xyz offsets.
        This is a pragmatic approximation for a construction excavator arm.

        Returns a length per joint in the chain. For revolute joints, this corresponds to
        the distance from the joint frame to the next joint frame (projected into X-Z).
        """
        joints = joints_override if joints_override is not None else self.chain_joints(base_link, tip_link)
        lengths: List[float] = []
        for jn in joints:
            ji = self.joints[jn]
            dx, _, dz = ji.origin_xyz[0], ji.origin_xyz[1], ji.origin_xyz[2]
            lengths.append(math.hypot(dx, dz))
        return lengths

    @staticmethod
    def wrap_to_pi(angle_rad: float) -> float:
        return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi
