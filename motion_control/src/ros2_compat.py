"""ROS2 导入适配与本地测试用 Twist 兼容实现。"""

from __future__ import annotations

import importlib
from typing import TYPE_CHECKING, Any


class _Vector3:
    def __init__(self):
        """初始化本地兼容消息的三轴分量，默认均为零。"""
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class _FallbackTwist:
    def __init__(self):
        """构造本地测试用 Twist，分别创建零值线速度和角速度向量。"""
        self.linear = _Vector3()
        self.angular = _Vector3()


if TYPE_CHECKING:
    # ROS2 安装在实车容器中；本地静态检查只声明当前使用的节点方法，
    # 并使用真实消息与兼容消息共有的向量字段。
    class Node:
        def __init__(self, node_name: str) -> None:
            """声明实车 ROS2 节点构造接口，仅供静态类型检查使用。"""
            ...
        def create_publisher(self, msg_type: type, topic: str, qos_profile: int) -> Any:
            """声明创建发布器的接口，运行时由容器中的 ROS2 SDK 实现。"""
            ...
        def destroy_node(self) -> None:
            """声明释放节点资源的接口。"""
            ...
        def get_namespace(self) -> str:
            """声明获取节点命名空间的接口。"""
            ...
        def get_name(self) -> str:
            """声明获取节点名称的接口。"""
            ...
        def get_logger(self) -> Any:
            """声明获取 ROS2 日志接口的方法。"""
            ...

    Twist = _FallbackTwist
    rclpy: Any
else:
    try:  # pragma: no cover - 此分支需在 TurboPi ROS2 环境执行。
        rclpy = importlib.import_module("rclpy")
        Twist = importlib.import_module("geometry_msgs.msg").Twist
        Node = importlib.import_module("rclpy.node").Node
    except ImportError:  # 本地测试环境未安装 ROS2 时使用兼容实现。
        rclpy = None
        Node = Any
        Twist = _FallbackTwist

__all__ = ["Node", "Twist", "rclpy"]
