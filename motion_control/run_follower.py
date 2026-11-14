#!/usr/bin/env python3
"""读取后处理 JSON，执行 TurboPi 原地转向后前进的路径控制。"""

from __future__ import annotations

import argparse
import os
import signal
import sys
from typing import Sequence

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from core import (  # noqa: E402
    RunBatch,
    STATUS_FAILED,
    STATUS_SUCCESS,
    make_error,
    make_result,
    metric_payload,
    require_success,
    write_result,
)
from motion_control.src.ros2_interface import (  # noqa: E402
    CmdVelPreflight,
    ExecutionReport,
    MotionSafetyError,
    Ros2PathExecutor,
)

MODULE = "motion_control"


def _load_config(path: str) -> dict:
    """读取运动控制 YAML 配置并返回字典。"""
    with open(path, encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}
    return raw.get("motion_control", raw)


def _load_segments(path: str) -> tuple[list[dict], dict]:
    """要求后处理结果成功，并提取其中可执行路径段及相关配置。"""
    result = require_success(path, module="path_postprocess")
    segments = result["outputs"].get("segments")
    if not isinstance(segments, list) or not segments:
        raise ValueError("path_postprocess result is missing non-empty outputs.segments")
    return segments, result.get("parameters", {})


def _positive_float(config: dict, name: str, default: float) -> float:
    """读取配置值并转为浮点数，未提供时使用默认值；非正数抛出 ValueError。"""
    value = float(config.get(name, default))
    if value <= 0:
        raise ValueError(f"{name} must be positive")
    return value


def _build_executor(config: dict) -> Ros2PathExecutor:
    """根据已解析配置构造 ROS2 路径执行器，并为缺省项提供默认值。"""
    return Ros2PathExecutor(
        turn_speed=_positive_float(config, "turn_speed", 1.0),
        move_speed=_positive_float(config, "move_speed", 0.5),
        grid_size=_positive_float(config, "grid_size", 0.3),
        timer_period=_positive_float(config, "timer_period", 0.1),
        odom_topic=config.get("odom_topic", "/odom"),
        initial_heading=config.get("initial_heading", "north"),
        cmd_vel_topic=config.get("cmd_vel_topic", "/cmd_vel"),
        empirical_forward_speed_m_s=_positive_float(
            config,
            "empirical_forward_speed_m_s",
            0.30,
        ),
        zero_transition_s=_positive_float(
            config,
            "zero_transition_s",
            float(config.get("timer_period", 0.1)),
        ),
        stop_publish_count=int(config.get("stop_publish_count", 10)),
        stop_publish_period_s=float(config.get("stop_publish_period_s", 0.02)),
        communication_buffer_s=float(config.get("communication_buffer_s", 0.1)),
    )


def _confirm_start(*, assume_yes: bool, input_stream=sys.stdin) -> None:
    """检查显式启动确认，或通过终端确认后才允许进入运动执行。"""
    if assume_yes:
        return
    if not input_stream.isatty():
        raise MotionSafetyError("未收到人工确认；非交互环境需显式传入 --yes")
    answer = input("确认启动小车运动？输入 yes 继续: ").strip().lower()
    if answer != "yes":
        raise MotionSafetyError("用户未确认启动运动")


def _preflight_payload(preflight: CmdVelPreflight | None) -> dict:
    """将 /cmd_vel 预检结果转换为 JSON 字典，未执行预检时返回空字典。"""
    if preflight is None:
        return {}
    return {
        "topic": preflight.topic,
        "subscribers": list(preflight.subscribers),
        "publishers": list(preflight.publishers),
        "chassis_subscriber_seen": preflight.chassis_subscriber_seen,
        "other_publishers": list(preflight.other_publishers),
        "confirmed": preflight.confirmed,
        "warnings": list(preflight.warnings),
    }


def _print_preflight(preflight: CmdVelPreflight) -> None:
    """向终端输出预检话题、订阅者、发布者及警告信息。"""
    print(f"[Follower] /cmd_vel 预检话题: {preflight.topic}")
    print(f"[Follower] /cmd_vel 订阅者: {list(preflight.subscribers)}")
    print(f"[Follower] /cmd_vel 发布者: {list(preflight.publishers)}")
    if preflight.warnings:
        for warning in preflight.warnings:
            print(f"[Follower] 预检警告: {warning}", file=sys.stderr)


def _success_result(
    *,
    run_id: str,
    postprocess_result: str,
    report: ExecutionReport,
    preflight: CmdVelPreflight,
    config: dict,
) -> dict:
    """将完成状态、停车状态、执行指标及预检信息封装为成功结果。"""
    return make_result(
        MODULE,
        run_id,
        STATUS_SUCCESS,
        inputs={"postprocess_result": os.path.abspath(postprocess_result)},
        outputs={
            "preflight": _preflight_payload(preflight),
            "execution": {
                "segment_count": report.segment_count,
                "command_count": report.command_count,
                "open_loop": report.open_loop,
                "completed": report.completed,
                "stopped": report.stopped,
            },
            "metrics": metric_payload(
                {
                    "segment_count": report.segment_count,
                    "command_count": report.command_count,
                    "open_loop": report.open_loop,
                },
                {
                    "segment_count": "count",
                    "command_count": "count",
                    "open_loop": "boolean",
                },
            ),
        },
        parameters=config,
    )


def _failure_result(
    *,
    run_id: str,
    postprocess_result: str | None,
    stage: str,
    exc: BaseException,
    suggestion: str,
    code: str = "MOTION_CONTROL_ERROR",
    config: dict | None = None,
    preflight: CmdVelPreflight | None = None,
) -> dict:
    """将异常或用户中断封装为带输入、阶段与恢复建议的运动失败结果。"""
    input_ref = os.path.abspath(postprocess_result) if postprocess_result else "<missing>"
    return make_result(
        MODULE,
        run_id,
        STATUS_FAILED,
        inputs={"postprocess_result": input_ref},
        outputs={"preflight": _preflight_payload(preflight)},
        parameters=config or {},
        errors=[
            make_error(
                code,
                module=MODULE,
                stage=stage,
                error_type=type(exc).__name__,
                reason=str(exc),
                input_ref=input_ref,
                suggestion=suggestion,
            )
        ],
    )


def _write_result_with_log(batch: RunBatch, result: dict, output_path: str) -> None:
    """写入运动模块结果，并将对应日志路径纳入结果记录。"""
    log_path = batch.log_path(MODULE)
    with open(log_path, "a", encoding="utf-8") as log:
        log.write(
            f"run_id={batch.run_id} status={result['status']} "
            f"input={result['inputs'].get('postprocess_result', '<missing>')}\n"
        )
    result["logs"] = [log_path]
    write_result(output_path, result)
    batch.update_stage(
        MODULE,
        result["status"],
        result=output_path,
        error=result["errors"] if result["status"] == STATUS_FAILED else None,
    )
    batch.finish(result["status"])


def main(argv: Sequence[str] | None = None) -> int:
    """运行运动控制 CLI，完成输入校验、话题预检及启动确认后执行路径段。

    结束、中断或异常时先执行停车，再销毁节点和幂等关闭 ROS2，保留本次结果与日志。"""
    parser = argparse.ArgumentParser(
        description="Drive TurboPi with turn -> zero -> forward path segments"
    )
    parser.add_argument(
        "--postprocess-result",
        help="path_postprocess success result JSON containing outputs.segments",
    )
    parser.add_argument(
        "--config",
        default=os.path.join(os.path.dirname(__file__), "config.yaml"),
    )
    parser.add_argument(
        "--allow-other-publishers",
        action="store_true",
        help="确认其他 /cmd_vel 发布者已被安全隔离或允许共存",
    )
    parser.add_argument(
        "--no-require-chassis-subscriber",
        action="store_true",
        help="调试用：不要求预检看到 /mecanum_chassis_node 订阅 /cmd_vel",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="确认已检查预检信息并允许启动运动",
    )
    parser.add_argument(
        "--stop-only",
        action="store_true",
        help="仅连接底盘并重复发布零速度后退出",
    )
    parser.add_argument("--output", help="motion_control result.json 输出路径")
    parser.add_argument("--run-root", default="runs")
    parser.add_argument("--run-id")
    args = parser.parse_args(argv)

    executor: Ros2PathExecutor | None = None
    batch = RunBatch(root=args.run_root, run_id=args.run_id).start()
    batch.register_module(MODULE)
    output_path = os.path.abspath(args.output) if args.output else os.path.join(
        batch.stage_dir(MODULE),
        "result.json",
    )
    config: dict = {}
    result_path = ""
    preflight: CmdVelPreflight | None = None

    def _raise_keyboard_interrupt(_sig, _frame):
        """将信号处理转为 KeyboardInterrupt，交由统一退出流程先停车再清理。"""
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, _raise_keyboard_interrupt)
    # SSH/terminal disconnect and service shutdown should also stop the car
    # before exiting, not just Ctrl+C.
    for signal_name in ("SIGTERM", "SIGHUP"):
        signal_value = getattr(signal, signal_name, None)
        if signal_value is not None:
            try:
                signal.signal(signal_value, _raise_keyboard_interrupt)
            except (OSError, ValueError):
                pass

    try:
        config = _load_config(args.config)
        if args.stop_only:
            executor = _build_executor(config)
            executor.setup_ros2()
            executor.stop_robot()
            return 0
        result_path = args.postprocess_result or config.get("postprocess_result")
        if not result_path:
            raise ValueError(
                "missing --postprocess-result or motion_control.postprocess_result"
            )
        if not os.path.isabs(result_path):
            result_path = os.path.abspath(os.path.join(ROOT, result_path))
        segments, postprocess_params = _load_segments(result_path)
        estimated_time = sum(float(item.get("estimated_time_s", 0.0)) for item in segments)
        print(f"[Follower] 读取路径段: {result_path}")
        print(f"[Follower] 路径段数: {len(segments)}")
        print(f"[Follower] 后处理参数: initial_heading={postprocess_params.get('initial_heading', '<unknown>')}")
        print(
            "[Follower] 运动模式: turn, /odom 不可用时使用 timer 开环估算；"
            "经验速度只用于粗略时间控制。"
        )
        print(f"[Follower] 后处理预计时间: {estimated_time:.2f}s")

        executor = _build_executor(config)
        executor.setup_ros2()
        preflight = executor.preflight_check(
            allow_other_publishers=args.allow_other_publishers,
            require_chassis_subscriber=not args.no_require_chassis_subscriber,
        )
        _print_preflight(preflight)
        if not preflight.confirmed:
            raise MotionSafetyError("; ".join(preflight.warnings))
        _confirm_start(assume_yes=args.yes)
        report = executor.execute_segments(
            segments,
            require_preflight=False,
        )
        print(
            f"[Follower] 执行完成: completed={report.completed}, "
            f"commands={report.command_count}, stopped={report.stopped}"
        )
        if not (report.completed and report.stopped):
            raise MotionSafetyError("运动控制未完成或停车状态未确认")
        result = _success_result(
            run_id=batch.run_id,
            postprocess_result=result_path,
            report=report,
            preflight=preflight,
            config=config,
        )
        _write_result_with_log(batch, result, output_path)
        return 0
    except MotionSafetyError as exc:
        print(f"[Follower] 安全预检失败，未启动运动: {exc}", file=sys.stderr)
        result = _failure_result(
            run_id=batch.run_id,
            postprocess_result=result_path,
            stage="preflight",
            exc=exc,
            suggestion="确认 /cmd_vel 底盘订阅者、停止其他速度发布者，并人工确认后重试",
            code="MOTION_PREFLIGHT_FAILED",
            config=config,
            preflight=preflight,
        )
        _write_result_with_log(batch, result, output_path)
        return 2
    except KeyboardInterrupt:
        print("\n[Follower] 手动中断，已请求先停车再退出。")
        result = _failure_result(
            run_id=batch.run_id,
            postprocess_result=result_path,
            stage="execute",
            exc=KeyboardInterrupt("user interrupted motion control"),
            suggestion="确认小车已停车后按需重新运行运动控制",
            code="MOTION_INTERRUPTED",
            config=config,
            preflight=preflight,
        )
        _write_result_with_log(batch, result, output_path)
        return 130
    except Exception as exc:
        print(f"[Follower] 执行出错: {exc}", file=sys.stderr)
        result = _failure_result(
            run_id=batch.run_id,
            postprocess_result=result_path,
            stage="execute",
            exc=exc,
            suggestion="检查输入 JSON、ROS2 环境和运动控制配置后重试",
            config=config,
            preflight=preflight,
        )
        _write_result_with_log(batch, result, output_path)
        return 2
    finally:
        if executor is not None:
            executor.close()


if __name__ == "__main__":
    raise SystemExit(main())
