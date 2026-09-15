"""Deterministic Fake ROS2 seam for motion-control acceptance checks."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Sequence

from .ros2_interface import (
    CmdVelPreflight,
    ExecutionReport,
    Ros2PathExecutor,
)

CommandTriple = tuple[float, float, float]
ZERO_COMMAND: CommandTriple = (0.0, 0.0, 0.0)


def _command_payload(commands: Sequence[CommandTriple]) -> list[dict[str, float]]:
    return [
        {
            "linear_x": linear_x,
            "linear_y": linear_y,
            "angular_z": angular_z,
        }
        for linear_x, linear_y, angular_z in commands
    ]


@dataclass(frozen=True)
class SimulatedEndpoint:
    """A minimal ROS graph endpoint description."""

    node_name: str
    node_namespace: str = "/"


class SimulatedClock:
    """Virtual clock used to avoid wall-clock waits during acceptance runs."""

    def __init__(self) -> None:
        self.elapsed_s = 0.0

    def sleep(self, seconds: float) -> None:
        self.elapsed_s += float(seconds)


class SimulatedPublisher:
    """Capture published Twist values as immutable numeric triples."""

    def __init__(self, events: list[str] | None = None) -> None:
        self.messages: list[CommandTriple] = []
        self.events = events if events is not None else []

    def publish(self, message: Any) -> None:
        command = (
            float(message.linear.x),
            float(message.linear.y),
            float(message.angular.z),
        )
        self.messages.append(command)
        self.events.append("publish_zero" if command == ZERO_COMMAND else "publish_motion")


class InterruptingClock(SimulatedClock):
    """Raise KeyboardInterrupt once after the first published command sleeps."""

    def __init__(self) -> None:
        super().__init__()
        self.interrupted = False

    def sleep(self, seconds: float) -> None:
        self.elapsed_s += float(seconds)
        if not self.interrupted:
            self.interrupted = True
            raise KeyboardInterrupt


class FailingPublisher(SimulatedPublisher):
    """Raise one runtime error on the first non-zero command."""

    def __init__(self, events: list[str] | None = None) -> None:
        super().__init__(events=events)
        self.failed = False

    def publish(self, message: Any) -> None:
        super().publish(message)
        command = self.messages[-1]
        if not self.failed and command != ZERO_COMMAND:
            self.failed = True
            raise RuntimeError("simulated publisher failure")


class SimulatedNode:
    """Fake ROS2 node exposing only the graph APIs used by the executor."""

    def __init__(self, events: list[str] | None = None) -> None:
        self.subscribers = [SimulatedEndpoint("mecanum_chassis_node")]
        self.publishers = [SimulatedEndpoint("path_follower")]
        self.destroyed = False
        self.events = events if events is not None else []

    def get_subscriptions_info_by_topic(self, _topic: str) -> list[SimulatedEndpoint]:
        return list(self.subscribers)

    def get_publishers_info_by_topic(self, _topic: str) -> list[SimulatedEndpoint]:
        return list(self.publishers)

    def get_namespace(self) -> str:
        return "/"

    def get_name(self) -> str:
        return "path_follower"

    def destroy_node(self) -> None:
        self.destroyed = True
        self.events.append("destroy_node")


@dataclass(frozen=True)
class MotionSimulation:
    """Public, JSON-ready result of a Fake ROS2 motion run."""

    report: ExecutionReport
    preflight: CmdVelPreflight
    commands: tuple[CommandTriple, ...]
    elapsed_s: float
    node_destroyed: bool
    events: tuple[str, ...]

    def to_payload(self) -> dict:
        return {
            "preflight": {
                "topic": self.preflight.topic,
                "subscribers": list(self.preflight.subscribers),
                "publishers": list(self.preflight.publishers),
                "chassis_subscriber_seen": self.preflight.chassis_subscriber_seen,
                "other_publishers": list(self.preflight.other_publishers),
                "confirmed": self.preflight.confirmed,
                "warnings": list(self.preflight.warnings),
            },
            "execution": {
                "segment_count": self.report.segment_count,
                "command_count": self.report.command_count,
                "open_loop": self.report.open_loop,
                "completed": self.report.completed,
                "stopped": self.report.stopped,
                "elapsed_s": self.elapsed_s,
                "node_destroyed": self.node_destroyed,
                "events": list(self.events),
            },
            "commands": _command_payload(self.commands),
        }


@dataclass(frozen=True)
class StopPathSimulation:
    """JSON-ready evidence for an interrupted or failed executor path."""

    exception_type: str
    commands: tuple[CommandTriple, ...]
    elapsed_s: float
    node_destroyed: bool
    events: tuple[str, ...]

    @property
    def stopped(self) -> bool:
        return bool(self.commands) and self.commands[-1] == ZERO_COMMAND

    @property
    def zero_before_destroy(self) -> bool:
        try:
            first_destroy = self.events.index("destroy_node")
        except ValueError:
            return False
        return "publish_zero" in self.events[:first_destroy]

    def to_payload(self) -> dict:
        return {
            "exception_type": self.exception_type,
            "stopped": self.stopped,
            "zero_before_destroy": self.zero_before_destroy,
            "elapsed_s": self.elapsed_s,
            "node_destroyed": self.node_destroyed,
            "events": list(self.events),
            "commands": _command_payload(self.commands),
        }


def simulate_segments(
    segments: Sequence[dict],
    *,
    turn_speed: float = 1.0,
    move_speed: float = 0.5,
    grid_size: float = 0.3,
    timer_period: float = 0.1,
    initial_heading: str | float = "north",
    empirical_forward_speed_m_s: float = 0.30,
    zero_transition_s: float | None = None,
    stop_publish_count: int = 10,
    stop_publish_period_s: float = 0.02,
    communication_buffer_s: float = 0.1,
) -> MotionSimulation:
    """Run path segments through the real executor against a Fake ROS2 graph."""
    clock = SimulatedClock()
    events: list[str] = []
    node = SimulatedNode(events=events)
    publisher = SimulatedPublisher(events=events)
    executor = Ros2PathExecutor(
        turn_speed=turn_speed,
        move_speed=move_speed,
        grid_size=grid_size,
        timer_period=timer_period,
        initial_heading=initial_heading,
        empirical_forward_speed_m_s=empirical_forward_speed_m_s,
        zero_transition_s=zero_transition_s,
        stop_publish_count=stop_publish_count,
        stop_publish_period_s=stop_publish_period_s,
        communication_buffer_s=communication_buffer_s,
        sleep=clock.sleep,
    )

    try:
        report = executor.execute_segments(
            segments,
            node=node,
            pub=publisher,
            require_preflight=True,
        )
    finally:
        executor.close()

    preflight = report.preflight
    if preflight is None:
        raise RuntimeError("motion simulation did not produce a preflight report")
    return MotionSimulation(
        report=report,
        preflight=preflight,
        commands=tuple(publisher.messages),
        elapsed_s=clock.elapsed_s,
        node_destroyed=node.destroyed,
        events=tuple(events),
    )


def simulate_keyboard_interrupt_stop(
    segments: Sequence[dict],
) -> StopPathSimulation:
    """Verify the executor publishes zero speed after Ctrl+C-like interruption."""
    clock = InterruptingClock()
    events: list[str] = []
    node = SimulatedNode(events=events)
    publisher = SimulatedPublisher(events=events)
    executor = Ros2PathExecutor(
        timer_period=0.1,
        stop_publish_count=2,
        stop_publish_period_s=0.0,
        communication_buffer_s=0.0,
        sleep=clock.sleep,
    )
    exception_type = "none"
    try:
        try:
            executor.execute_segments(segments, node=node, pub=publisher)
        except KeyboardInterrupt as exc:
            exception_type = type(exc).__name__
    finally:
        executor.close()
    return StopPathSimulation(
        exception_type=exception_type,
        commands=tuple(publisher.messages),
        elapsed_s=clock.elapsed_s,
        node_destroyed=node.destroyed,
        events=tuple(events),
    )


def simulate_exception_stop(
    segments: Sequence[dict],
) -> StopPathSimulation:
    """Verify the executor publishes zero speed after a runtime exception."""
    clock = SimulatedClock()
    events: list[str] = []
    node = SimulatedNode(events=events)
    publisher = FailingPublisher(events=events)
    executor = Ros2PathExecutor(
        timer_period=0.1,
        stop_publish_count=2,
        stop_publish_period_s=0.0,
        communication_buffer_s=0.0,
        sleep=clock.sleep,
    )
    exception_type = "none"
    try:
        try:
            executor.execute_segments(segments, node=node, pub=publisher)
        except RuntimeError as exc:
            exception_type = type(exc).__name__
    finally:
        executor.close()
    return StopPathSimulation(
        exception_type=exception_type,
        commands=tuple(publisher.messages),
        elapsed_s=clock.elapsed_s,
        node_destroyed=node.destroyed,
        events=tuple(events),
    )


__all__ = [
    "FailingPublisher",
    "InterruptingClock",
    "MotionSimulation",
    "SimulatedClock",
    "SimulatedEndpoint",
    "SimulatedNode",
    "SimulatedPublisher",
    "StopPathSimulation",
    "simulate_exception_stop",
    "simulate_keyboard_interrupt_stop",
    "simulate_segments",
]
