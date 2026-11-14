"""模块 CLI 子进程执行及 Ctrl+C 中断转发。"""

from __future__ import annotations

import os
import signal
import subprocess
import sys
from typing import Any, Sequence

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _subprocess_runner(command: Sequence[str]) -> int:
    """执行模块子进程并返回退出码，收到 Ctrl+C 时向子进程转发中断。

    运动模块额外遵循安全停车处理，避免顶层退出后子进程继续发布速度。"""
    kwargs: dict[str, Any] = {"cwd": PROJECT_ROOT}
    if sys.platform == "win32":
        kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
    else:
        kwargs["start_new_session"] = True

    process = subprocess.Popen(list(command), **kwargs)
    try:
        return process.wait()
    except KeyboardInterrupt:
        try:
            process.send_signal(
                signal.CTRL_C_EVENT if sys.platform == "win32" else signal.SIGINT
            )
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            if _is_motion_command(command):
                _run_stop_only(command)
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2)
        raise


def _is_motion_command(command: Sequence[str]) -> bool:
    """识别命令是否启动运动控制模块，以选择中断时的停车处理。"""
    return "-m" in command and "motion_control.run_follower" in command


def _run_stop_only(command: Sequence[str]) -> None:
    """启动仅停车子进程并限时等待；失败时输出诊断，必要时清理超时进程。"""
    stop_command = [
        sys.executable,
        "-m",
        "motion_control.run_follower",
        "--stop-only",
    ]
    if "--config" in command:
        index = command.index("--config")
        if index + 1 < len(command):
            stop_command.extend(["--config", command[index + 1]])
    kwargs: dict[str, Any] = {"cwd": PROJECT_ROOT}
    if sys.platform == "win32":
        kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
    else:
        kwargs["start_new_session"] = True
    stop_process = None
    try:
        stop_process = subprocess.Popen(stop_command, **kwargs)
        stop_process.wait(timeout=8)
    except (OSError, subprocess.TimeoutExpired) as exc:
        # 后续仍会清理原进程；ROS2 不可用时不能宣称仅停车命令已成功。
        print(
            f"[top-level] stop-only 失败: {exc}；请立即人工确认小车已停止",
            file=sys.stderr,
        )
        try:
            if stop_process is not None:
                stop_process.kill()
        except (UnboundLocalError, OSError):
            pass


__all__ = [
    "PROJECT_ROOT",
    "_is_motion_command",
    "_run_stop_only",
    "_subprocess_runner",
]
