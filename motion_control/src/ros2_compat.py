"""ROS2 import shim and Twist fallback for local tests."""

from __future__ import annotations

from typing import Any

try:  # pragma: no cover - exercised on the TurboPi ROS2 runtime.
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
except ImportError:  # Local tests run without ROS2 installed.
    rclpy = None
    Node = Any

    class _Vector3:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0

    class Twist:  # type: ignore[no-redef]
        def __init__(self):
            self.linear = _Vector3()
            self.angular = _Vector3()


__all__ = ["Node", "Twist", "rclpy"]
