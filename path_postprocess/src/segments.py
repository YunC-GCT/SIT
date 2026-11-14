"""从已校验路径或平滑曲线生成可执行路径段。"""

from __future__ import annotations

import math
from numbers import Real
from typing import Sequence

import numpy as np

from core import path_segment_payload

from .geometry import (
    RawPathError,
    _bresenham_points,
    _heading_degrees,
    _initial_heading_degrees,
    _signed_angle_difference,
    _validate_path,
)
from .smoothers import SmoothingError

class SegmentationError(ValueError):
    """配置的分段长度限制无法用当前路径表示时抛出的异常。"""

def _discretize_curve(
    curve: Sequence[Sequence[float]],
    *,
    grid_shape: tuple[int, int],
    start: tuple[int, int],
    goal: tuple[int, int],
) -> list[tuple[int, int]]:
    """校验曲线坐标并取整为栅格路径，移除连续重复点并固定起终点。"""
    rows, cols = grid_shape
    try:
        curve_values = list(curve)
    except TypeError as exc:
        raise SmoothingError("smoother output must be a sequence") from exc
    if not curve_values:
        raise SmoothingError("smoother returned no points")
    discrete = []
    for index, point in enumerate(curve_values):
        try:
            coordinates = tuple(point)
        except TypeError:
            coordinates = ()
        if (
            len(coordinates) != 2
            or any(
                not isinstance(value, Real)
                or isinstance(value, bool)
                or not math.isfinite(float(value))
                for value in coordinates
            )
        ):
            raise SmoothingError(f"smoother point {index} is not finite")
        row, col = float(coordinates[0]), float(coordinates[1])
        if not (0.0 <= row <= rows - 1 and 0.0 <= col <= cols - 1):
            raise SmoothingError(f"smoother curve is out of bounds at point {index}")
        rounded = (int(round(row)), int(round(col)))
        if not discrete or discrete[-1] != rounded:
            discrete.append(rounded)
    if not discrete:
        raise SmoothingError("smoother returned no usable points")
    if len(discrete) == 1 and start != goal:
        raise SmoothingError("smoother did not preserve distinct endpoints")
    discrete[0] = start
    discrete[-1] = goal
    return discrete

def _validate_sampled_curve(
    grid: np.ndarray,
    curve: Sequence[Sequence[float]],
    *,
    start: tuple[int, int],
    goal: tuple[int, int],
    sample_spacing_grid: float,
) -> None:
    """校验曲线的有限坐标、端点保持、采样间距和采样连接的碰撞情况。"""
    rows, cols = grid.shape
    try:
        points = [tuple(point) for point in curve]
    except TypeError as exc:
        raise SmoothingError("smoother output must be a sequence") from exc
    if not points:
        raise SmoothingError("smoother returned no points")
    if any(len(point) != 2 for point in points):
        raise SmoothingError("smoother output points must contain row and col")
    if not all(
        math.isfinite(float(value))
        for point in points
        for value in point
    ):
        raise SmoothingError("smoother output contains a non-finite coordinate")
    if not all(
        math.isclose(float(actual), float(expected), abs_tol=1e-9)
        for actual, expected in zip(points[0], start)
    ) or not all(
        math.isclose(float(actual), float(expected), abs_tol=1e-9)
        for actual, expected in zip(points[-1], goal)
    ):
        raise SmoothingError("smoother did not preserve endpoints")
    for point in points:
        if len(point) != 2:
            raise SmoothingError("smoother output points must contain row and col")
        point_row, point_col = float(point[0]), float(point[1])
        if not (0.0 <= point_row <= rows - 1 and 0.0 <= point_col <= cols - 1):
            raise RawPathError("sampled spline is out of bounds")
    for first, second in zip(points, points[1:]):
        first_row, first_col = float(first[0]), float(first[1])
        second_row, second_col = float(second[0]), float(second[1])
        distance = math.hypot(second_row - first_row, second_col - first_col)
        sample_count = max(
            1,
            math.ceil(distance / min(sample_spacing_grid, 0.25)),
        )
        for index in range(sample_count + 1):
            fraction = index / sample_count
            row = int(round(first_row + (second_row - first_row) * fraction))
            col = int(round(first_col + (second_col - first_col) * fraction))
            if not (0 <= row < rows and 0 <= col < cols):
                raise RawPathError("sampled spline is out of bounds")
            if grid[row, col] != 0:
                raise RawPathError("sampled spline connection is blocked")

def _expand_segments(
    path: Sequence[tuple[int, int]],
    max_segment_length_grid: float,
) -> list[tuple[int, int]]:
    """将超长整数路径边拆成符合段长限制的栅格连接点。"""
    expanded = [path[0]]
    for first, second in zip(path, path[1:]):
        distance = math.hypot(second[0] - first[0], second[1] - first[1])
        if distance <= max_segment_length_grid + 1e-9:
            expanded.append(second)
            continue
        split_points = _bresenham_points(first, second)
        cursor = 0
        while cursor < len(split_points) - 1:
            next_cursor = cursor + 1
            while next_cursor + 1 < len(split_points):
                candidate = split_points[next_cursor + 1]
                candidate_distance = math.hypot(
                    candidate[0] - split_points[cursor][0],
                    candidate[1] - split_points[cursor][1],
                )
                if candidate_distance > max_segment_length_grid + 1e-9:
                    break
                next_cursor += 1
            if next_cursor == cursor:
                raise SegmentationError(
                    "max_segment_length_grid is smaller than an integer grid step"
                )
            expanded.append(split_points[next_cursor])
            cursor = next_cursor
    return expanded

def _compact_path(
    expanded: Sequence[tuple[int, int]],
    max_segment_length_grid: float,
    *,
    corner_tolerance_deg: float = 10.0,
) -> list[tuple[int, int]]:
    """合并连续共线密集点，在转角处保留端点并限制执行段长度。

    使执行段数量由路径几何决定，避免 A* 密集点与采样算法稀疏点造成不同的停走频率。"""
    if len(expanded) < 3:
        return list(expanded)
    endpoints = [expanded[0]]
    segment_start = expanded[0]
    previous = expanded[0]
    previous_direction: float | None = None
    for point in expanded[1:]:
        direction = _heading_degrees(previous, point)
        corner = (
            previous_direction is not None
            and abs(_signed_angle_difference(direction, previous_direction))
            > corner_tolerance_deg + 1e-9
        )
        if corner and endpoints[-1] != previous:
            endpoints.append(previous)
            segment_start = previous
        distance = math.hypot(
            point[0] - segment_start[0],
            point[1] - segment_start[1],
        )
        if distance > max_segment_length_grid + 1e-9:
            if endpoints[-1] != previous:
                endpoints.append(previous)
            segment_start = previous
        previous_direction = direction
        previous = point
    if endpoints[-1] != expanded[-1]:
        endpoints.append(expanded[-1])
    return endpoints

def _execution_endpoints(
    grid: np.ndarray,
    path: Sequence[tuple[int, int]],
    max_segment_length_grid: float,
) -> tuple[list[tuple[int, int]], bool]:
    """生成归一化执行端点；若合并后的直线穿障，则回退到安全连接。"""
    expanded = _expand_segments(path, max_segment_length_grid)
    compacted = _compact_path(expanded, max_segment_length_grid)
    if compacted == expanded:
        return compacted, False
    try:
        _validate_path(grid, compacted)
    except RawPathError:
        return expanded, True
    return compacted, False

def _resample_curve(
    curve: Sequence[Sequence[float]],
    max_segment_length_grid: float,
) -> list[tuple[float, float]]:
    """按弧长对平滑曲线重新采样，返回浮点执行端点。

    端点保留在连续曲线上，避免每个采样点取整造成阶梯路径及距离高估。"""
    points = [(float(point[0]), float(point[1])) for point in curve]
    if len(points) < 2:
        return points
    cumulative = [0.0]
    for first, second in zip(points, points[1:]):
        cumulative.append(
            cumulative[-1]
            + math.hypot(second[0] - first[0], second[1] - first[1])
        )
    total = cumulative[-1]
    if total <= 0:
        return [points[0], points[-1]]

    step = min(float(max_segment_length_grid), total)
    targets = []
    distance = 0.0
    while distance < total - 1e-9:
        targets.append(distance)
        distance += step
    targets.append(total)

    endpoints: list[tuple[float, float]] = []
    index = 0
    for target in targets:
        while index + 1 < len(cumulative) and cumulative[index + 1] < target:
            index += 1
        if index + 1 >= len(points):
            endpoint = points[-1]
        else:
            span = cumulative[index + 1] - cumulative[index]
            fraction = 0.0 if span <= 0 else (target - cumulative[index]) / span
            endpoint = (
                points[index][0] + (points[index + 1][0] - points[index][0]) * fraction,
                points[index][1] + (points[index + 1][1] - points[index][1]) * fraction,
            )
        if not endpoints or math.hypot(
            endpoint[0] - endpoints[-1][0],
            endpoint[1] - endpoints[-1][1],
        ) > 1e-9:
            endpoints.append(endpoint)
    if endpoints[-1] != points[-1]:
        endpoints.append(points[-1])
    return endpoints

def _validate_curve_chords(
    grid: np.ndarray,
    endpoints: Sequence[tuple[float, float]],
) -> bool:
    """检查重采样端点之间的直线弦均在地图空闲区域内。"""
    rows, cols = grid.shape
    for first, second in zip(endpoints, endpoints[1:]):
        distance = math.hypot(second[0] - first[0], second[1] - first[1])
        sample_count = max(1, math.ceil(distance / 0.25))
        for index in range(sample_count + 1):
            fraction = index / sample_count
            row = int(round(first[0] + (second[0] - first[0]) * fraction))
            col = int(round(first[1] + (second[1] - first[1]) * fraction))
            if not (0 <= row < rows and 0 <= col < cols):
                return False
            if grid[row, col] != 0:
                return False
    return True

def _make_segments(
    grid: np.ndarray,
    path: Sequence[tuple[int, int]],
    *,
    curve: Sequence[Sequence[float]] | None = None,
    grid_size_m: float,
    max_segment_length_grid: float,
    move_speed_m_s: float,
    turn_speed_rad_s: float,
    initial_heading: str | float,
) -> tuple[list[dict], bool]:
    """将执行端点转换为含方向、转角、米制距离与预计时间的标准路径段。"""
    endpoints: Sequence[Sequence[float]]
    normalization_fallback = False
    if curve is not None:
        curve_endpoints = _resample_curve(curve, max_segment_length_grid)
        if len(curve_endpoints) >= 2 and _validate_curve_chords(grid, curve_endpoints):
            endpoints = curve_endpoints
        else:
            endpoints, normalization_fallback = _execution_endpoints(
                grid,
                path,
                max_segment_length_grid,
            )
    else:
        endpoints, normalization_fallback = _execution_endpoints(
            grid,
            path,
            max_segment_length_grid,
        )
    previous_heading = _initial_heading_degrees(initial_heading)
    segments = []
    for index, (start, end) in enumerate(zip(endpoints, endpoints[1:])):
        distance_grid = math.hypot(end[0] - start[0], end[1] - start[1])
        if distance_grid <= 0:
            continue
        direction = _heading_degrees(start, end)
        turn_angle = _signed_angle_difference(direction, previous_heading)
        distance_m = distance_grid * grid_size_m
        estimated_time = (
            distance_m / move_speed_m_s
            + math.radians(abs(turn_angle)) / turn_speed_rad_s
        )
        segments.append(
            path_segment_payload(
                index,
                start=start,
                end=end,
                direction_deg=direction,
                turn_angle_deg=turn_angle,
                grid_distance=distance_grid,
                distance_m=distance_m,
                estimated_time_s=estimated_time,
                motion_type="turn",
            )
        )
        previous_heading = direction
    return segments, normalization_fallback


__all__ = [
    "SegmentationError",
    "_compact_path",
    "_discretize_curve",
    "_execution_endpoints",
    "_expand_segments",
    "_make_segments",
    "_resample_curve",
    "_validate_curve_chords",
    "_validate_sampled_curve",
]
