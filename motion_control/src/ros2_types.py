"""运动控制共用的执行结果与安全异常类型。"""

from __future__ import annotations

from dataclasses import dataclass

class MotionSafetyError(RuntimeError):
    """ROS2 话题图未满足底盘运动预检要求时抛出的异常。"""


@dataclass(frozen=True)
class CmdVelPreflight:
    topic: str
    subscribers: tuple[str, ...]
    publishers: tuple[str, ...]
    chassis_subscriber_seen: bool
    other_publishers: tuple[str, ...]
    confirmed: bool
    warnings: tuple[str, ...]


@dataclass(frozen=True)
class ExecutionReport:
    segment_count: int
    command_count: int
    open_loop: bool
    completed: bool
    stopped: bool
    preflight: CmdVelPreflight | None = None


__all__ = [
    "CmdVelPreflight",
    "ExecutionReport",
    "MotionSafetyError",
]
