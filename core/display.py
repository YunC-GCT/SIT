"""图形显示及交互终端可用性的运行环境检查。"""

from __future__ import annotations

import os
import sys


def display_available() -> bool:
    """根据平台和显示环境变量判断是否预期可打开图形窗口。"""
    if sys.platform.startswith("win") or sys.platform == "darwin":
        return True
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


def interactive_terminal() -> bool:
    """判断标准输入、输出是否都连接到交互终端。"""
    return bool(sys.stdin.isatty() and sys.stdout.isatty())


__all__ = ["display_available", "interactive_terminal"]
