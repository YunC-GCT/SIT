#!/usr/bin/env python3
"""Map processing entry point.

Use ``python -m map_pipeline.json_pipeline`` or ``python map_pipeline/json_pipeline.py``.
The implementation lives in :mod:`map_pipeline.process`,
:mod:`map_pipeline.interaction`, and :mod:`map_pipeline.cli`.
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
