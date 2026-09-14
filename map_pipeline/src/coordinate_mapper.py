"""
将原图像素坐标映射为栅格坐标
"""

from numbers import Integral
from typing import Iterable, Tuple

from .grid_map import GridMap


def map_pixel_to_grid(pixel_x: int, pixel_y: int, grid_map: GridMap):
    """
    根据栅格地图的 cell 尺寸将像素坐标转换为栅格索引。
    pixel_x: 列方向 (对应 col)
    pixel_y: 行方向 (对应 row)
    """
    col = int(pixel_x / grid_map.cell_width)
    row = int(pixel_y / grid_map.cell_height)
    # 边界裁剪
    col = max(0, min(grid_map.cols - 1, col))
    row = max(0, min(grid_map.rows - 1, row))
    return row, col


def validate_grid_point(
    grid_map: GridMap,
    point: Iterable[int],
    name: str = "point",
) -> Tuple[int, int]:
    """Validate one row/col point without changing the requested location."""
    try:
        row, col = point
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} 必须是 (row, col) 坐标") from exc

    if isinstance(row, bool) or isinstance(col, bool):
        raise ValueError(f"{name} 必须是整数坐标")
    if not isinstance(row, Integral) or not isinstance(col, Integral):
        raise ValueError(f"{name} 必须是整数坐标")
    row, col = int(row), int(col)
    if not (0 <= row < grid_map.rows and 0 <= col < grid_map.cols):
        raise ValueError(
            f"{name} 超出地图范围: ({row}, {col}), "
            f"shape=({grid_map.rows},{grid_map.cols})"
        )
    if not grid_map.is_free(row, col):
        raise ValueError(f"{name} 落在障碍栅格 ({row}, {col})，请重新选择")
    return row, col


def validate_grid_points(
    grid_map: GridMap,
    start: Iterable[int],
    goal: Iterable[int],
) -> Tuple[Tuple[int, int], Tuple[int, int]]:
    """Validate start and goal as free grid cells."""
    return (
        validate_grid_point(grid_map, start, "start"),
        validate_grid_point(grid_map, goal, "goal"),
    )
