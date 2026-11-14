"""支持原地转向后前进及安全停车的 ROS2 路径执行器。"""

from __future__ import annotations

import math
import time
from typing import Any, Callable, Protocol, Sequence

from .ros2_compat import Node, Twist, rclpy
from .ros2_geometry import (
    compute_segment_turns,
    heading_degrees,
    split_turn_angle_deg,
)
from .ros2_twist import CHASSIS_NODE_NAME, CMD_VEL_TOPIC, make_twist
from .ros2_types import CmdVelPreflight, ExecutionReport, MotionSafetyError

class ExecutorNode(Protocol):
    """真实 ROS2 节点与离线模拟器共用的最小节点接口。"""

    def get_namespace(self) -> str:
        """返回节点命名空间；真实节点和离线节点都需提供此接口。"""
        ...
    def get_name(self) -> str:
        """返回节点名称，供识别本节点的速度发布者使用。"""
        ...
    def destroy_node(self) -> None:
        """释放节点资源；执行器保证先完成停车流程再调用。"""
        ...


class Ros2PathExecutor:
    def __init__(
        self,
        turn_speed: float = 1.0,
        move_speed: float = 0.5,
        grid_size: float = 0.3,
        timer_period: float = 0.1,
        odom_topic: str = "/odom",
        initial_heading: str | float = "north",
        *,
        cmd_vel_topic: str = CMD_VEL_TOPIC,
        empirical_forward_speed_m_s: float = 0.30,
        zero_transition_s: float | None = None,
        stop_publish_count: int = 10,
        stop_publish_period_s: float = 0.02,
        communication_buffer_s: float = 0.1,
        sleep: Callable[[float], None] | None = None,
    ):
        """配置速度指令、计时周期、初始朝向和停车发送策略，初始化执行状态。

        经验前进速度用于开环时间估算，不等于已验证的通用速度标定；odom_topic 当前仅保留。"""
        if turn_speed <= 0:
            raise ValueError("turn_speed must be positive")
        if move_speed <= 0:
            raise ValueError("move_speed must be positive")
        if empirical_forward_speed_m_s <= 0:
            raise ValueError("empirical_forward_speed_m_s must be positive")
        if timer_period <= 0:
            raise ValueError("timer_period must be positive")
        if not isinstance(cmd_vel_topic, str) or not cmd_vel_topic.startswith("/"):
            raise ValueError("cmd_vel_topic must be an absolute ROS2 topic")
        heading_degrees(initial_heading)
        self.turn_speed = float(turn_speed)
        self.move_speed = float(move_speed)
        self.grid_size = float(grid_size)
        self.timer_period = float(timer_period)
        self.odom_topic = odom_topic
        self.initial_heading = initial_heading
        self.cmd_vel_topic = cmd_vel_topic
        self.empirical_forward_speed_m_s = float(empirical_forward_speed_m_s)
        self.zero_transition_s = (
            float(zero_transition_s)
            if zero_transition_s is not None
            else float(timer_period)
        )
        self.stop_publish_count = int(stop_publish_count)
        self.stop_publish_period_s = float(stop_publish_period_s)
        self.communication_buffer_s = float(communication_buffer_s)
        self.sleep = sleep or time.sleep

        self.node: ExecutorNode | None = None
        self.pub: Any = None
        self.done = False
        self._shutdown_done = False
        self._movement_enabled = False
        self._stopped = False
        self._last_report: ExecutionReport | None = None
        self._node_destroyed = False
        self._forward_ticks_due = 0.0

    def setup_ros2(self):
        """按需初始化 ROS2，创建路径跟随节点及 /cmd_vel 发布器并返回。"""
        if rclpy is None:
            raise RuntimeError("ROS2 Python packages are not available")
        if not rclpy.ok():
            rclpy.init(args=None)
        node = Node("path_follower")
        self.node = node
        self._node_destroyed = False
        self.pub = node.create_publisher(Twist, self.cmd_vel_topic, 1)
        return self.node, self.pub

    def execute_segments(
        self,
        segments: Sequence[dict],
        *,
        node: ExecutorNode | None = None,
        pub: Any = None,
        require_preflight: bool = True,
        allow_other_publishers: bool = False,
        require_chassis_subscriber: bool = True,
    ) -> ExecutionReport:
        """执行转向与前进路径段，可注入模拟节点及发布器进行离线验证。

        默认先做话题预检；正常结束和异常退出均执行停车，异常继续向调用方传播。"""
        if node is not None:
            self.node = node
        if pub is not None:
            self.pub = pub
        if self.node is None or self.pub is None:
            self.setup_ros2()

        preflight = None
        self.done = False
        self._movement_enabled = False
        self._stopped = False
        self._forward_ticks_due = 0.0
        command_count = 0
        completed = False
        try:
            if require_preflight:
                preflight = self.preflight_check(
                    allow_other_publishers=allow_other_publishers,
                    require_chassis_subscriber=require_chassis_subscriber,
                )
                if not preflight.confirmed:
                    raise MotionSafetyError("; ".join(preflight.warnings))
            self._movement_enabled = True
            self._log_info(
                "运动控制使用 timer 开环估算: "
                f"经验前进速度 {self.empirical_forward_speed_m_s:.2f} m/s, "
                "不宣称精确闭环。"
            )
            command_count = self._run_turn_segments(segments)
            self.done = True
            completed = True
        except KeyboardInterrupt:
            self._log_info("收到 Ctrl+C，准备先停车再退出程序...")
            raise
        except Exception:
            raise
        finally:
            self._movement_enabled = False
            self.stop_robot()
        report = self._record_report(
            segments,
            command_count,
            completed=completed,
            preflight=preflight,
        )
        return report

    def preflight_check(
        self,
        *,
        allow_other_publishers: bool = False,
        require_chassis_subscriber: bool = True,
    ) -> CmdVelPreflight:
        """查询速度话题的底盘订阅者和其他发布者，按安全要求确认或拒绝启动。"""
        subscribers = self._topic_endpoint_names("subscriptions")
        publishers = self._topic_endpoint_names("publishers")
        self_name = self._node_name()
        other_publishers = tuple(name for name in publishers if name != self_name)
        chassis_seen = any(
            name == CHASSIS_NODE_NAME or name.endswith(f"/{CHASSIS_NODE_NAME}")
            for name in subscribers
        )
        warnings = []
        if require_chassis_subscriber and not chassis_seen:
            warnings.append(
                f"{self.cmd_vel_topic} 未确认到底盘订阅者 /{CHASSIS_NODE_NAME}"
            )
        if other_publishers and not allow_other_publishers:
            warnings.append(
                f"{self.cmd_vel_topic} 存在其他发布者: {', '.join(other_publishers)}"
            )
        confirmed = not warnings
        report = CmdVelPreflight(
            topic=self.cmd_vel_topic,
            subscribers=subscribers,
            publishers=publishers,
            chassis_subscriber_seen=chassis_seen,
            other_publishers=other_publishers,
            confirmed=confirmed,
            warnings=tuple(warnings),
        )
        self._log_preflight(report)
        return report

    def stop_robot(self) -> None:
        """禁用后续运动，重复发送零速度并留出通信缓冲，成功完成后记录停车状态。

        允许重复调用；停车发送不依赖后续 ROS2 shutdown 是否成功。"""
        if self.pub is not None:
            zero = make_twist()
            published = 0
            try:
                for _ in range(max(1, self.stop_publish_count)):
                    self.pub.publish(zero)
                    published += 1
                    if self.stop_publish_period_s > 0:
                        self.sleep(self.stop_publish_period_s)
                if self.communication_buffer_s > 0:
                    self.sleep(self.communication_buffer_s)
            except Exception as exc:
                self._log_warn(
                    f"停车命令发布失败（已发布 {published} 次）: {exc}"
                )
                self._stopped = False
                return
        else:
            self._stopped = False
            return
        self._stopped = True
        self._log_info("小车已停止（Twist 零速度已重复发出）。")

    def close(self) -> None:
        """依次停车、销毁节点并关闭 ROS2，使用 finally 保证资源清理顺序。"""
        try:
            self.stop_robot()
        finally:
            try:
                if (
                    self.node is not None
                    and not self._node_destroyed
                    and hasattr(self.node, "destroy_node")
                ):
                    self.node.destroy_node()
                    self._node_destroyed = True
            finally:
                self.shutdown_once()

    def shutdown_once(self) -> None:
        """仅关闭一次 ROS2 上下文，容忍已关闭类错误以避免重复 shutdown。"""
        if self._shutdown_done:
            return
        self._shutdown_done = True
        if rclpy is None:
            return
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except RuntimeError as exc:
            message = str(exc)
            if "shutdown" not in message.lower() and "context" not in message.lower():
                raise

    def _run_turn_segments(self, segments: Sequence[dict]) -> int:
        """逐段先原地转向、零速过渡，再沿底盘 X 正方向前进并累计发布次数。"""
        command_count = 0
        turns = compute_segment_turns(segments, initial_heading=self.initial_heading)
        for segment, turn in zip(segments, turns):
            index = int(segment.get("index", command_count))
            for partial_turn in split_turn_angle_deg(turn):
                command_count += self._publish_for_duration(
                    # 地图航向顺时针递增；在 X 向前、Y 向左的底盘坐标系中，
                    # ROS angular.z 以逆时针为正，因此发布时需要反号。
                    make_twist(angular_z=-_sign(partial_turn) * self.turn_speed),
                    math.radians(abs(partial_turn)) / self.turn_speed,
                    state=f"segment {index} rotate {partial_turn:.1f}deg",
                )
                command_count += self._publish_zero_transition(index)
            command_count += self._publish_zero_transition(index)
            distance_m = float(segment.get("distance_m", self.grid_size))
            command_count += self._publish_forward_distance(
                distance_m,
                state=f"segment {index} forward {distance_m:.3f}m open-loop",
            )
            command_count += self._publish_zero_transition(index)
        return command_count

    def _publish_forward_distance(self, distance_m: float, *, state: str) -> int:
        """根据距离及经验前进速度累计应发布的控制周期数，再发送前进指令。

        跨路径段保留不足一周期的余量，避免每段独立取整造成总行驶时间偏差。"""
        if not self._movement_enabled:
            return 0
        target_s = max(0.0, float(distance_m)) / self.empirical_forward_speed_m_s
        self._forward_ticks_due += target_s / self.timer_period
        count = max(0, int(math.floor(self._forward_ticks_due + 0.5)))
        if count <= 0:
            self._log_info(
                f"{state}: {target_s:.2f}s (carried {self._forward_ticks_due:.2f} ticks)"
            )
            return 0
        self._log_info(f"{state}: {target_s:.2f}s -> {count} ticks")
        published = 0
        for _ in range(count):
            if not self._movement_enabled:
                break
            self.pub.publish(make_twist(linear_x=self.move_speed))
            self.sleep(self.timer_period)
            published += 1
        self._forward_ticks_due -= published
        return published

    def _publish_for_duration(self, twist: Twist, duration_s: float, *, state: str) -> int:
        """按计时周期发送指定 Twist，运动被禁用时停止发布，返回本次周期计数。"""
        if not self._movement_enabled:
            return 0
        count = max(1, int(math.ceil(max(0.0, duration_s) / self.timer_period)))
        self._log_info(f"{state}: {duration_s:.2f}s")
        for _ in range(count):
            if not self._movement_enabled:
                break
            self.pub.publish(twist)
            self.sleep(self.timer_period)
        return count

    def _publish_zero_transition(self, segment_index: int) -> int:
        """发送指定时长的零速过渡，隔开转向与前进动作。"""
        return self._publish_for_duration(
            make_twist(),
            self.zero_transition_s,
            state=f"segment {segment_index} zero transition",
        )

    def _record_report(
        self,
        segments: Sequence[dict],
        command_count: int,
        *,
        completed: bool,
        preflight: CmdVelPreflight | None,
    ) -> ExecutionReport:
        """保存执行段数、命令数、完成和停车状态，返回执行报告。"""
        report = ExecutionReport(
            segment_count=len(segments),
            command_count=command_count,
            open_loop=True,
            completed=completed,
            stopped=self._stopped,
            preflight=preflight,
        )
        self._last_report = report
        return report

    def _topic_endpoint_names(self, kind: str) -> tuple[str, ...]:
        """查询话题端点并整理节点名称；查询失败时形成安全预检错误。"""
        if self.node is None:
            return ()
        method_name = (
            "get_subscriptions_info_by_topic"
            if kind == "subscriptions"
            else "get_publishers_info_by_topic"
        )
        method = getattr(self.node, method_name, None)
        if method is None:
            raise MotionSafetyError(
                f"当前 ROS2 node 不支持查询 {self.cmd_vel_topic} {kind}"
            )
        try:
            infos = method(self.cmd_vel_topic)
        except Exception as exc:
            raise MotionSafetyError(
                f"读取 {self.cmd_vel_topic} {kind} 失败，发布者/订阅者状态未确认: {exc}"
            ) from exc
        names = []
        for info in infos:
            name = getattr(info, "node_name", "")
            namespace = getattr(info, "node_namespace", "")
            full_name = _full_node_name(namespace, name)
            if full_name:
                names.append(full_name)
        return tuple(names)

    def _node_name(self) -> str:
        """读取执行器当前节点的命名空间及名称，组合为完整节点名。"""
        if self.node is None:
            return ""
        try:
            return _full_node_name(
                self.node.get_namespace(),
                self.node.get_name(),
            )
        except Exception:
            return ""

    def _log_preflight(self, report: CmdVelPreflight) -> None:
        """记录速度话题预检摘要及警告。"""
        self._log_info(
            f"预检 {report.topic}: subscribers={list(report.subscribers)}, "
            f"publishers={list(report.publishers)}"
        )
        for warning in report.warnings:
            self._log_warn(warning)

    def _log_info(self, message: str) -> None:
        """若节点提供日志器，则记录一般运行信息。"""
        logger = self._logger()
        if logger is not None:
            logger.info(message)

    def _log_warn(self, message: str) -> None:
        """若节点提供日志器，则记录警告信息。"""
        logger = self._logger()
        if logger is not None:
            logger.warn(message)

    def _logger(self):
        """获取可选节点日志器；无节点或模拟节点未提供日志器时返回 None。"""
        get_logger = getattr(self.node, "get_logger", None)
        if get_logger is not None:
            return get_logger()
        return None

def _sign(value: float) -> float:
    """返回数值符号，非负数返回 1.0，负数返回 -1.0。"""
    return 1.0 if value >= 0 else -1.0

def _full_node_name(namespace: str, name: str) -> str:
    """将命名空间和节点名组合为用于话题预检的完整节点名称。"""
    if not name:
        return ""
    namespace = namespace or "/"
    if namespace == "/":
        return f"/{name}"
    return f"{namespace.rstrip('/')}/{name}"


__all__ = ["Ros2PathExecutor"]
