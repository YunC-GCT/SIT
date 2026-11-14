#!/usr/bin/env python3
"""地图处理的兼容入口。

支持 python -m map_pipeline.json_pipeline 或直接执行本文件。
实现分别位于 map_pipeline.process、map_pipeline.interaction 和 map_pipeline.cli。
"""

from __future__ import annotations

import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from map_pipeline.cli import _record_failed_batch, main
from map_pipeline.interaction import _interactive_points, _select_terminal_points
from map_pipeline.process import prepare_grid_map, process

__all__ = [
    "_interactive_points",
    "_record_failed_batch",
    "_select_terminal_points",
    "main",
    "prepare_grid_map",
    "process",
]

if __name__ == "__main__":
    raise SystemExit(main())
