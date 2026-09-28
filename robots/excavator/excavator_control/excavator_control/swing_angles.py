"""Periodic angle arithmetic for site-frame excavator swing feedback."""

import math


def shortest_angle_error(target_rad: float, current_rad: float) -> float:
    """Signed shortest rotation to target, with a positive 180-degree tie."""
    error = (target_rad - current_rad + math.pi) % (2.0 * math.pi) - math.pi
    return math.pi if error == -math.pi else error
