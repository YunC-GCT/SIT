"""Twist construction and /cmd_vel constants."""

from __future__ import annotations

import math

from .ros2_compat import Twist

CMD_VEL_TOPIC = "/cmd_vel"
CHASSIS_NODE_NAME = "mecanum_chassis_node"

def make_twist(
    *,
    linear_x: float = 0.0,
    linear_y: float = 0.0,
    angular_z: float = 0.0,
) -> Twist:
    twist = Twist()
    twist.linear.x = float(linear_x)
    twist.linear.y = float(linear_y)
    twist.angular.z = float(angular_z)
    return twist

def is_zero_twist(twist: Twist) -> bool:
    return (
        math.isclose(float(twist.linear.x), 0.0, abs_tol=1e-12)
        and math.isclose(float(twist.linear.y), 0.0, abs_tol=1e-12)
        and math.isclose(float(twist.angular.z), 0.0, abs_tol=1e-12)
    )


__all__ = [
    "CHASSIS_NODE_NAME",
    "CMD_VEL_TOPIC",
    "is_zero_twist",
    "make_twist",
]
