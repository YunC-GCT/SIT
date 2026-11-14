#!/usr/bin/env python3
"""路径规划 JSON CLI 兼容入口。"""

from __future__ import annotations

import os
import sys

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(BASE_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from path_planner.json_pipeline import main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(main())
