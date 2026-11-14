"""Path geometry, normalization, and collision validation helpers."""

from __future__ import annotations

import math
from numbers import Integral
from typing import Any, Iterable, Sequence

import numpy as np

from .parameters import HEADINGS_DEG

class RawPathError(ValueError):
    """Raised when the planner path cannot be safely executed."""

def _normalize_grid(grid: Any) -> np.ndarray:
    normalized = np.asarray(grid)
    if normalized.ndim != 2 or 0 in normalized.shape:
        raise RawPathError("grid must be a non-empty two-dimensional array")
    return normalized

def _normalize_path(path: Iterable[Iterable[int]]) -> list[tuple[int, int]]:
    if path is None:
        raise RawPathError("raw path is missing")
    try:
        values = list(path)
    except TypeError as exc:
        raise RawPathError("raw path must be an array of row/col coordinates") from exc
    if not values:
        raise RawPathError("raw path is empty")

    normalized = []
    for index, point in enumerate(values):
        try:
            coordinates = tuple(point)
        except TypeError:
            coordinates = ()
        if (
            len(coordinates) != 2
            or any(
                not isinstance(value, Integral) or isinstance(value, bool)
                for value in coordinates
            )
        ):
            raise RawPathError(
                f"raw path point {index} must contain two integer coordinates"
            )
        normalized_point = (int(coordinates[0]), int(coordinates[1]))
        if not normalized or normalized[-1] != normalized_point:
            normalized.append(normalized_point)
    if not normalized:
        raise RawPathError("raw path contains no usable points")
    return normalized

def _bresenham_points(
    first: tuple[int, int],
    second: tuple[int, int],
) -> list[tuple[int, int]]:
    row, col = first
    target_row, target_col = second
    points = [first]
    delta_row = abs(target_row - row)
    delta_col = abs(target_col - col)
    step_row = 1 if row < target_row else -1
    step_col = 1 if col < target_col else -1
    error = delta_row - delta_col
    while (row, col) != (target_row, target_col):
        double_error = 2 * error
        if double_error > -delta_col:
            error -= delta_col
            row += step_row
        if double_error < delta_row:
            error += delta_row
            col += step_col
        points.append((row, col))
    return points

def _line_of_sight(
    grid: np.ndarray,
    first: tuple[int, int],
    second: tuple[int, int],
) -> bool:
    rows, cols = grid.shape
    for row, col in _bresenham_points(first, second):
        if not (0 <= row < rows and 0 <= col < cols):
            return False
        if grid[row, col] != 0:
            return False
    return True

def _validate_path(
    grid: np.ndarray,
    path: Sequence[tuple[int, int]],
    *,
    expected_start: tuple[int, int] | None = None,
    expected_goal: tuple[int, int] | None = None,
) -> None:
    if expected_start is not None and path[0] != expected_start:
        raise RawPathError("raw path does not start at the map start")
    if expected_goal is not None and path[-1] != expected_goal:
        raise RawPathError("raw path does not end at the map goal")
    for point in path:
        if not (0 <= point[0] < grid.shape[0] and 0 <= point[1] < grid.shape[1]):
            raise RawPathError(f"raw path point is out of bounds: {point}")
        if grid[point] != 0:
            raise RawPathError(f"raw path point is an obstacle: {point}")
    for first, second in zip(path, path[1:]):
        if not _line_of_sight(grid, first, second):
            raise RawPathError(f"raw path connection is blocked: {first} -> {second}")

def _path_length(path: Sequence[Sequence[float]]) -> float:
    return sum(
        math.hypot(second[0] - first[0], second[1] - first[1])
        for first, second in zip(path, path[1:])
    )

def _heading_degrees(point: Sequence[float], target: Sequence[float]) -> float:
    delta_row = target[0] - point[0]
    delta_col = target[1] - point[1]
    return math.degrees(math.atan2(delta_col, -delta_row)) % 360.0

def _signed_angle_difference(target: float, current: float) -> float:
    difference = (target - current + 180.0) % 360.0 - 180.0
    if math.isclose(difference, -180.0, abs_tol=1e-9):
        return 180.0
    return difference

def _initial_heading_degrees(value: str | float) -> float:
    if isinstance(value, str):
        return HEADINGS_DEG[value]
    return float(value) % 360.0

def _normalize_endpoint(value: Iterable[int], name: str) -> tuple[int, int]:
    try:
        point = tuple(value)
    except TypeError as exc:
        raise RawPathError(f"{name} must be a row/col coordinate") from exc
    if (
        len(point) != 2
        or any(
            not isinstance(item, Integral) or isinstance(item, bool)
            for item in point
        )
    ):
        raise RawPathError(f"{name} must contain two integer coordinates")
    return int(point[0]), int(point[1])


__all__ = [
    "RawPathError",
    "_bresenham_points",
    "_heading_degrees",
    "_initial_heading_degrees",
    "_line_of_sight",
    "_normalize_endpoint",
    "_normalize_grid",
    "_normalize_path",
    "_path_length",
    "_signed_angle_difference",
    "_validate_path",
]
