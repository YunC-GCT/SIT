"""Grid, start/goal, and planner path validation."""

from __future__ import annotations

from numbers import Integral, Real
from typing import Any, Iterable

import numpy as np

from .planner_factory import PlannerParameterError

def _point(value: Iterable[int], name: str) -> tuple[int, int]:
    if not isinstance(value, (tuple, list)) or len(value) != 2:
        raise PlannerParameterError(f"{name} 必须是 [row, col] 坐标")
    if any(
        not isinstance(item, Integral) or isinstance(item, bool)
        for item in value
    ):
        raise PlannerParameterError(f"{name} 的 row/col 必须是整数")
    return int(value[0]), int(value[1])

def _validate_grid(grid: Any) -> np.ndarray:
    normalized = np.asarray(grid)
    if normalized.ndim != 2:
        raise PlannerParameterError("grid 必须是二维栅格数组")
    if normalized.shape[0] == 0 or normalized.shape[1] == 0:
        raise PlannerParameterError("grid 不能为空")
    return normalized

def _validate_inputs(
    grid: Any,
    start: Iterable[int],
    goal: Iterable[int],
    grid_size_m: float,
) -> tuple[np.ndarray, tuple[int, int], tuple[int, int]]:
    normalized_grid = _validate_grid(grid)
    normalized_start = _point(start, "start")
    normalized_goal = _point(goal, "goal")
    rows, cols = normalized_grid.shape
    for name, point in (("start", normalized_start), ("goal", normalized_goal)):
        row, col = point
        if not (0 <= row < rows and 0 <= col < cols):
            raise PlannerParameterError(
                f"{name} 坐标超出地图范围: {point}, shape={normalized_grid.shape}"
            )
        if normalized_grid[point] != 0:
            raise PlannerParameterError(f"{name} 位于障碍栅格: {point}")
    if (
        not isinstance(grid_size_m, Real)
        or isinstance(grid_size_m, bool)
        or grid_size_m <= 0
    ):
        raise PlannerParameterError("grid_size_m 必须是正数")
    return normalized_grid, normalized_start, normalized_goal

def _normalize_path(
    path: Iterable[Iterable[int]],
    grid: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
) -> list[tuple[int, int]]:
    normalized = []
    rows, cols = grid.shape
    if path is None:
        path = []
    for index, point in enumerate(path):
        try:
            row, col = point
        except (TypeError, ValueError) as exc:
            raise ValueError(f"规划器路径第 {index} 项不是 row/col 坐标") from exc
        if (
            not isinstance(row, Integral)
            or isinstance(row, bool)
            or not isinstance(col, Integral)
            or isinstance(col, bool)
        ):
            raise ValueError(f"规划器路径第 {index} 项坐标必须是整数")
        point_tuple = (int(row), int(col))
        if not (0 <= point_tuple[0] < rows and 0 <= point_tuple[1] < cols):
            raise ValueError(f"规划器路径第 {index} 项越界: {point_tuple}")
        if grid[point_tuple] != 0:
            raise ValueError(f"规划器路径第 {index} 项穿过障碍: {point_tuple}")
        normalized.append(point_tuple)
    if normalized and (normalized[0] != start or normalized[-1] != goal):
        raise ValueError("规划器路径必须以 start 开始并以 goal 结束")
    return normalized


__all__ = [
    "_normalize_path",
    "_point",
    "_validate_grid",
    "_validate_inputs",
]
