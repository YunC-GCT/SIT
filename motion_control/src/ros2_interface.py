"""ROS2 motion-control interface compatibility facade."""

from __future__ import annotations

from .ros2_compat import Node, Twist, rclpy
from .ros2_executor import Ros2PathExecutor
from .ros2_geometry import (
    DIR_EAST,
    DIR_NORTH,
    DIR_SOUTH,
    DIR_WEST,
    HEADING_TO_DEG,
    HEADING_TO_DIR,
    compute_segment_turns,
    compute_turn_angles,
    delta_to_dir,
    estimate_runtime,
    heading_degrees,
    normalize_angle_deg,
    split_turn_angle_deg,
)
from .ros2_twist import (
    CHASSIS_NODE_NAME,
    CMD_VEL_TOPIC,
    is_zero_twist,
    make_twist,
)
from .ros2_types import CmdVelPreflight, ExecutionReport, MotionSafetyError

__all__ = [
    "CHASSIS_NODE_NAME",
    "CMD_VEL_TOPIC",
    "CmdVelPreflight",
    "DIR_EAST",
    "DIR_NORTH",
    "DIR_SOUTH",
    "DIR_WEST",
    "ExecutionReport",
    "HEADING_TO_DEG",
    "HEADING_TO_DIR",
    "MotionSafetyError",
    "Node",
    "Ros2PathExecutor",
    "Twist",
    "compute_segment_turns",
    "compute_turn_angles",
    "delta_to_dir",
    "estimate_runtime",
    "heading_degrees",
    "is_zero_twist",
    "make_twist",
    "normalize_angle_deg",
    "rclpy",
    "split_turn_angle_deg",
]
