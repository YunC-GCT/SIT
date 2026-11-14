"""ROS2 import shim and Twist fallback for local tests."""

from __future__ import annotations

import importlib
from typing import TYPE_CHECKING, Any


class _Vector3:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class _FallbackTwist:
    def __init__(self):
        self.linear = _Vector3()
        self.angular = _Vector3()


if TYPE_CHECKING:
    # ROS2 is installed only in the vehicle container. Its Node API is dynamic;
    # local checks use the vector fields shared by real and fallback messages.
    class Node:
        def __init__(self, node_name: str) -> None: ...
        def create_publisher(self, msg_type: type, topic: str, qos_profile: int) -> Any: ...
        def destroy_node(self) -> None: ...
        def get_namespace(self) -> str: ...
        def get_name(self) -> str: ...
        def get_logger(self) -> Any: ...

    Twist = _FallbackTwist
    rclpy: Any
else:
    try:  # pragma: no cover - exercised on the TurboPi ROS2 runtime.
        rclpy = importlib.import_module("rclpy")
        Twist = importlib.import_module("geometry_msgs.msg").Twist
        Node = importlib.import_module("rclpy.node").Node
    except ImportError:  # Local tests run without ROS2 installed.
        rclpy = None
        Node = Any
        Twist = _FallbackTwist

__all__ = ["Node", "Twist", "rclpy"]
