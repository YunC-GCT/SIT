"""用于运动控制验收的确定性模拟 ROS2 接口。"""

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
    """将线速度 X/Y 与角速度 Z 三元组封装为 JSON 命令记录。"""
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
    """模拟 ROS2 话题端点的最小描述。"""

    node_name: str
    node_namespace: str = "/"


class SimulatedClock:
    """验收使用的虚拟时钟，以推进计数代替真实等待。"""

    def __init__(self) -> None:
        """初始化模拟时钟，计时不依赖真实等待。"""
        self.elapsed_s = 0.0

    def sleep(self, seconds: float) -> None:
        """推进虚拟时间，替代执行器中的真实 sleep。"""
        self.elapsed_s += float(seconds)


class SimulatedPublisher:
    """把已发布 Twist 的受控分量记录为不可变数值三元组。"""

    def __init__(self, events: list[str] | None = None) -> None:
        """初始化模拟发布器，用于记录执行器发送的速度命令。"""
        self.messages: list[CommandTriple] = []
        self.events = events if events is not None else []

    def publish(self, message: Any) -> None:
        """提取并记录 Twist 的受控速度分量，不连接真实底盘。"""
        command = (
            float(message.linear.x),
            float(message.linear.y),
            float(message.angular.z),
        )
        self.messages.append(command)
        self.events.append("publish_zero" if command == ZERO_COMMAND else "publish_motion")


class InterruptingClock(SimulatedClock):
    """首次等待时触发一次 KeyboardInterrupt，用于验证中断停车。"""

    def __init__(self) -> None:
        """初始化模拟时钟，并设置首次等待时触发一次中断的状态。"""
        super().__init__()
        self.interrupted = False

    def sleep(self, seconds: float) -> None:
        """推进虚拟时间，并仅在首次等待时抛出 KeyboardInterrupt 以验证停车。"""
        self.elapsed_s += float(seconds)
        if not self.interrupted:
            self.interrupted = True
            raise KeyboardInterrupt


class FailingPublisher(SimulatedPublisher):
    """首次非零命令发布时触发一次运行异常。"""

    def __init__(self, events: list[str] | None = None) -> None:
        """设置模拟发布故障，并保留命令和事件记录。"""
        super().__init__(events=events)
        self.failed = False

    def publish(self, message: Any) -> None:
        """先记录命令，再在首次非零速度发布时抛出一次运行异常以验证停车。"""
        super().publish(message)
        command = self.messages[-1]
        if not self.failed and command != ZERO_COMMAND:
            self.failed = True
            raise RuntimeError("simulated publisher failure")


class SimulatedNode:
    """提供执行器所需话题图查询方法的模拟 ROS2 节点。"""

    def __init__(self, events: list[str] | None = None) -> None:
        """建立模拟底盘订阅者和当前发布者，并初始化销毁状态及事件列表。"""
        self.subscribers = [SimulatedEndpoint("mecanum_chassis_node")]
        self.publishers = [SimulatedEndpoint("path_follower")]
        self.destroyed = False
        self.events = events if events is not None else []

    def get_subscriptions_info_by_topic(self, _topic: str) -> list[SimulatedEndpoint]:
        """返回模拟话题订阅端点副本，供执行器预检使用。"""
        return list(self.subscribers)

    def get_publishers_info_by_topic(self, _topic: str) -> list[SimulatedEndpoint]:
        """返回模拟话题发布端点副本，供检查其他速度发布者使用。"""
        return list(self.publishers)

    def get_namespace(self) -> str:
        """返回模拟节点的根命名空间。"""
        return "/"

    def get_name(self) -> str:
        """返回模拟路径跟随节点名。"""
        return "path_follower"

    def destroy_node(self) -> None:
        """标记模拟节点已销毁并记录事件，便于核对先停车后退出。"""
        self.destroyed = True
        self.events.append("destroy_node")


@dataclass(frozen=True)
class MotionSimulation:
    """可转换为 JSON 的模拟运动执行结果。"""

    report: ExecutionReport
    preflight: CmdVelPreflight
    commands: tuple[CommandTriple, ...]
    elapsed_s: float
    node_destroyed: bool
    events: tuple[str, ...]

    def to_payload(self) -> dict:
        """将运动模拟报告、命令、计时和销毁状态转换为 JSON 载荷。"""
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
    """可转换为 JSON 的中断或异常停车验证记录。"""

    exception_type: str
    commands: tuple[CommandTriple, ...]
    elapsed_s: float
    node_destroyed: bool
    events: tuple[str, ...]

    @property
    def stopped(self) -> bool:
        """根据已记录命令判断退出路径最终是否发送零速。"""
        return bool(self.commands) and self.commands[-1] == ZERO_COMMAND

    @property
    def zero_before_destroy(self) -> bool:
        """检查事件顺序，确认零速发布发生在节点销毁之前。"""
        try:
            first_destroy = self.events.index("destroy_node")
        except ValueError:
            return False
        return "publish_zero" in self.events[:first_destroy]

    def to_payload(self) -> dict:
        """将异常停车模拟的异常、命令和事件顺序转换为 JSON 载荷。"""
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
    """将真实执行器接到模拟 ROS2 图和时钟，运行路径段并返回可检查的记录。"""
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
    """模拟 Ctrl+C 中断，记录零速发布及节点销毁顺序。"""
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
    """模拟运行时故障，记录异常传播、停车和资源清理结果。"""
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
