"""Shared helpers for map point and corner pickers."""

from __future__ import annotations

import tempfile
from typing import Tuple, Any

import cv2
import numpy as np

from .coordinate_mapper import map_pixel_to_grid
from .grid_map import GridMap

class PointSelectionCancelled(RuntimeError):
    """用户取消了起终点选择，或提前关闭了选点窗口。"""

def _display_geometry(
    width: int,
    height: int,
    max_width: int = 1000,
    max_height: int = 700,
) -> tuple[float, int, int]:
    """Return a display scale and dimensions that use the available canvas."""
    if width < 1 or height < 1:
        raise ValueError("显示图像尺寸必须为正数")
    if max_width < 1 or max_height < 1:
        raise ValueError("显示窗口尺寸必须为正数")

    scale = min(max_width / width, max_height / height)
    return scale, max(1, int(round(width * scale))), max(
        1, int(round(height * scale))
    )

def _normalize_key(key: Any) -> int:
    """Normalize OpenCV backend key codes to the low-byte key value."""
    try:
        value = int(key)
    except (TypeError, ValueError):
        return -1
    return value if value < 0 else value & 0xFF

def _is_confirm_key(key: Any) -> bool:
    """Recognize Enter/Return codes emitted by common OpenCV backends."""
    try:
        value = int(key)
    except (TypeError, ValueError):
        return False
    if _normalize_key(value) in (10, 13):
        return True
    return value in {
        0x01000004,  # Qt::Key_Return
        0x01000005,  # Qt::Key_Enter
    }

def _picker_image(image: np.ndarray) -> tuple[np.ndarray, float, int, int]:
    height, width = image.shape[:2]
    scale, display_width, display_height = _display_geometry(width, height)
    interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_NEAREST
    return (
        cv2.resize(
            image,
            (display_width, display_height),
            interpolation=interpolation,
        ),
        scale,
        display_width,
        display_height,
    )

def _to_rgb(image: np.ndarray) -> np.ndarray:
    if image.ndim == 2:
        return cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
    if image.shape[2] == 4:
        return cv2.cvtColor(image, cv2.COLOR_BGRA2RGB)
    return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

def _write_temp_ppm(rgb_image: np.ndarray) -> str:
    height, width = rgb_image.shape[:2]
    handle = tempfile.NamedTemporaryFile(delete=False, suffix=".ppm")
    try:
        handle.write(f"P6\n{width} {height}\n255\n".encode("ascii"))
        handle.write(np.ascontiguousarray(rgb_image).tobytes())
        return handle.name
    finally:
        handle.close()

def _snap_to_grid_center(
    point: Tuple[int, int],
    grid_map: GridMap | None,
) -> Tuple[int, int]:
    """将点击位置吸附到其栅格的中心像素。"""
    if grid_map is None:
        return point

    row, col = map_pixel_to_grid(point[0], point[1], grid_map)
    max_y, max_x = grid_map.img_shape[0] - 1, grid_map.img_shape[1] - 1
    # Use the lower integer pixel of the mathematical center.  This keeps
    # one-pixel cells stable: round(1.5) would move the second cell to x=2.
    center_x = int((col + 0.5) * grid_map.cell_width)
    center_y = int((row + 0.5) * grid_map.cell_height)
    return (
        max(0, min(max_x, center_x)),
        max(0, min(max_y, center_y)),
    )

def _marker_radius(scale: float, grid_map: GridMap | None) -> int:
    """Return a small marker that does not visually cover a whole cell."""
    if grid_map is None:
        return max(1, min(4, int(round(2 * scale))))
    display_cell = min(grid_map.cell_width, grid_map.cell_height) * scale
    return max(1, min(3, int(display_cell / 3)))


__all__ = [
    "PointSelectionCancelled",
    "_display_geometry",
    "_is_confirm_key",
    "_marker_radius",
    "_normalize_key",
    "_picker_image",
    "_snap_to_grid_center",
    "_to_rgb",
    "_write_temp_ppm",
]
