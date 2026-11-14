"""Subprocess execution with safe Ctrl+C handoff for module CLIs."""

from __future__ import annotations

import os
import signal
import subprocess
import sys
from typing import Any, Sequence

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _subprocess_runner(command: Sequence[str]) -> int:
    """Run one module CLI while preserving safe Ctrl+C delivery to children."""
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
    return "-m" in command and "motion_control.run_follower" in command


def _run_stop_only(command: Sequence[str]) -> None:
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
    try:
        stop_process = subprocess.Popen(stop_command, **kwargs)
        stop_process.wait(timeout=8)
    except (OSError, subprocess.TimeoutExpired) as exc:
        # The original process is still terminated below; no successful
        # stop-only command can be claimed when ROS2 is unavailable.
        print(
            f"[top-level] stop-only 失败: {exc}；请立即人工确认小车已停止",
            file=sys.stderr,
        )
        try:
            stop_process.kill()
        except (UnboundLocalError, OSError):
            pass


__all__ = [
    "PROJECT_ROOT",
    "_is_motion_command",
    "_run_stop_only",
    "_subprocess_runner",
]
