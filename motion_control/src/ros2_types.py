"""Shared motion-control result and safety types."""

from __future__ import annotations

from dataclasses import dataclass

class MotionSafetyError(RuntimeError):
    """Raised when the ROS graph is not safe enough to drive the chassis."""


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
