"""Twist 消息构造及 /cmd_vel 相关常量。"""

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
    """新建零值 Twist 后设置 X/Y 线速度及 Z 角速度，避免残留上次动作分量。"""
    twist = Twist()
    twist.linear.x = float(linear_x)
    twist.linear.y = float(linear_y)
    twist.angular.z = float(angular_z)
    return twist

def is_zero_twist(twist: Twist) -> bool:
    """检查受控的 X/Y 线速度和 Z 角速度是否均在容差内为零。"""
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
