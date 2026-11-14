"""Runtime checks for graphical display and interactive terminal use."""

from __future__ import annotations

import os
import sys


def display_available() -> bool:
    """Return whether a GUI backend can reasonably be expected to open windows."""
    if sys.platform.startswith("win") or sys.platform == "darwin":
        return True
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


def interactive_terminal() -> bool:
    """Return whether both input and output are attached to a terminal."""
    return bool(sys.stdin.isatty() and sys.stdout.isatty())


__all__ = ["display_available", "interactive_terminal"]
