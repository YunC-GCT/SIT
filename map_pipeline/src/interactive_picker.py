"""Point and corner selection entry points.

Implementation modules: picker_common, picker_opencv, picker_tk, picker_quad.
"""

from __future__ import annotations

from typing import Tuple

import numpy as np

from .grid_map import GridMap
from .picker_common import (
    PointSelectionCancelled,
    _display_geometry,
    _is_confirm_key,
    _normalize_key,
    _snap_to_grid_center,
)
from .picker_opencv import _pick_points_opencv
from .picker_quad import pick_quad
from .picker_tk import _pick_points_tk

def pick_points(
    image: np.ndarray,
    window_name: str = "Select Start & Goal",
    grid_map: GridMap | None = None,
    backend: str = "auto",
) -> Tuple[Tuple[int, int], Tuple[int, int]]:
    """Select start and goal points using Tk on Windows when available."""
    if backend not in {"auto", "tk", "opencv"}:
        raise ValueError("backend must be auto, tk, or opencv")
    if not isinstance(image, np.ndarray) or image.ndim < 2:
        raise ValueError("选点图像必须是至少二维的 numpy 数组")

    if backend in {"auto", "tk"}:
        try:
            return _pick_points_tk(image.copy(), window_name, grid_map)
        except PointSelectionCancelled:
            raise
        except (ImportError, OSError, RuntimeError):
            if backend == "tk":
                raise
    return _pick_points_opencv(image.copy(), window_name, grid_map)


__all__ = [
    "PointSelectionCancelled",
    "_display_geometry",
    "_is_confirm_key",
    "_normalize_key",
    "_snap_to_grid_center",
    "pick_points",
    "pick_quad",
]
