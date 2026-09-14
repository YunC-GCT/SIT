"""Path smoothing, validation, and executable segment generation."""

from __future__ import annotations

import math
import os
from numbers import Integral, Real
from typing import Any, Iterable, Sequence

import numpy as np

from core import (
    STATUS_FAILED,
    STATUS_SUCCESS,
    make_error,
    make_result,
    metric_payload,
    path_payload,
    path_segment_payload,
)

from .smoothers import NaturalCubicSplineSmoother, PathSmoother, SmoothingError

MODULE = "path_postprocess"
DEFAULT_PARAMETERS = {
    "smooth": True,
    "sample_spacing_grid": 0.25,
    "max_segment_length_grid": 2.0,
    "move_speed_m_s": 0.30,
    "turn_speed_rad_s": 2.0,
    "initial_heading": "north",
}
HEADINGS_DEG = {
    "north": 0.0,
    "east": 90.0,
    "south": 180.0,
    "west": 270.0,
}


class PathPostprocessParameterError(ValueError):
    """Raised when post-processing parameters are invalid."""


class RawPathError(ValueError):
    """Raised when the planner path cannot be safely executed."""


class SegmentationError(ValueError):
    """Raised when the configured segment limit cannot be represented."""


def resolve_postprocess_parameters(
    parameters: dict[str, Any] | None = None,
    *,
    grid_size_m: float = 0.3,
) -> dict[str, Any]:
    """Validate and return effective post-processing parameters."""
    if parameters is not None and not isinstance(parameters, dict):
        raise PathPostprocessParameterError("parameters must be a JSON object")
    values = dict(DEFAULT_PARAMETERS)
    provided = dict(parameters or {})
    aliases = {
        "sample_spacing_m",
        "max_segment_length_m",
    }
    unknown = sorted(set(provided) - set(values) - aliases)
    if unknown:
        raise PathPostprocessParameterError(
            f"unsupported post-processing parameters: {', '.join(unknown)}"
        )
    if (
        not isinstance(grid_size_m, Real)
        or isinstance(grid_size_m, bool)
        or not math.isfinite(float(grid_size_m))
        or grid_size_m <= 0
    ):
        raise PathPostprocessParameterError("grid_size_m must be positive")

    if "sample_spacing_m" in provided and "sample_spacing_grid" not in provided:
        try:
            provided["sample_spacing_grid"] = (
                float(provided["sample_spacing_m"]) / float(grid_size_m)
            )
        except (TypeError, ValueError, ZeroDivisionError) as exc:
            raise PathPostprocessParameterError(
                "sample_spacing_m must be a positive number"
            ) from exc
    if (
        "max_segment_length_m" in provided
        and "max_segment_length_grid" not in provided
    ):
        try:
            provided["max_segment_length_grid"] = (
                float(provided["max_segment_length_m"]) / float(grid_size_m)
            )
        except (TypeError, ValueError, ZeroDivisionError) as exc:
            raise PathPostprocessParameterError(
                "max_segment_length_m must be a positive number"
            ) from exc
    values.update(provided)

    if not isinstance(values["smooth"], bool):
        raise PathPostprocessParameterError("smooth must be boolean")
    for name in (
        "sample_spacing_grid",
        "max_segment_length_grid",
        "move_speed_m_s",
        "turn_speed_rad_s",
    ):
        value = values[name]
        if (
            not isinstance(value, Real)
            or isinstance(value, bool)
            or not math.isfinite(float(value))
            or value <= 0
        ):
            raise PathPostprocessParameterError(f"{name} must be positive")
        values[name] = float(value)
    heading = values["initial_heading"]
    if isinstance(heading, str):
        if heading.lower() not in HEADINGS_DEG:
            raise PathPostprocessParameterError(
                "initial_heading must be north, east, south, or west"
            )
        values["initial_heading"] = heading.lower()
    elif isinstance(heading, Real) and not isinstance(heading, bool):
        if not math.isfinite(float(heading)):
            raise PathPostprocessParameterError("initial_heading must be finite")
        values["initial_heading"] = float(heading) % 360.0
    else:
        raise PathPostprocessParameterError(
            "initial_heading must be a compass name or degree value"
        )
    return values


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


def _path_length(path: Sequence[Sequence[Real]]) -> float:
    return sum(
        math.hypot(second[0] - first[0], second[1] - first[1])
        for first, second in zip(path, path[1:])
    )


def _discretize_curve(
    curve: Sequence[Sequence[Real]],
    *,
    grid_shape: tuple[int, int],
    start: tuple[int, int],
    goal: tuple[int, int],
) -> list[tuple[int, int]]:
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
    curve: Sequence[Sequence[Real]],
    *,
    start: tuple[int, int],
    goal: tuple[int, int],
    sample_spacing_grid: float,
) -> None:
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


def _heading_degrees(point: tuple[int, int], target: tuple[int, int]) -> float:
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


def _make_segments(
    path: Sequence[tuple[int, int]],
    *,
    grid_size_m: float,
    max_segment_length_grid: float,
    move_speed_m_s: float,
    turn_speed_rad_s: float,
    initial_heading: str | float,
) -> list[dict]:
    expanded = _expand_segments(path, max_segment_length_grid)
    previous_heading = _initial_heading_degrees(initial_heading)
    segments = []
    for index, (start, end) in enumerate(zip(expanded, expanded[1:])):
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
    return segments


def _failure_result(
    *,
    run_id: str,
    input_ref: str,
    parameters: dict[str, Any],
    code: str,
    stage: str,
    error: Exception,
    suggestion: str,
) -> dict:
    return make_result(
        MODULE,
        run_id,
        STATUS_FAILED,
        inputs={"planner_result": os.path.abspath(input_ref) if input_ref else "<memory>"},
        outputs={"raw_path": path_payload([])},
        parameters=parameters,
        errors=[
            make_error(
                code,
                module=MODULE,
                stage=stage,
                error_type=type(error).__name__,
                reason=str(error),
                input_ref=input_ref or "<memory>",
                suggestion=suggestion,
            )
        ],
    )


def process_path(
    grid: Any,
    raw_path: Iterable[Iterable[int]],
    *,
    grid_size_m: float = 0.3,
    start: Iterable[int] | None = None,
    goal: Iterable[int] | None = None,
    parameters: dict[str, Any] | None = None,
    smoother: PathSmoother | None = None,
    run_id: str = "standalone",
    input_ref: str = "",
) -> dict:
    """Convert one planner path into a validated path-postprocess result."""
    effective_parameters: dict[str, Any] = (
        dict(parameters) if isinstance(parameters, dict) else {}
    )
    try:
        normalized_grid = _normalize_grid(grid)
        effective_parameters = resolve_postprocess_parameters(
            parameters,
            grid_size_m=grid_size_m,
        )
        normalized_path = _normalize_path(raw_path)
        expected_start = _normalize_endpoint(start, "start") if start is not None else None
        expected_goal = _normalize_endpoint(goal, "goal") if goal is not None else None
        _validate_path(
            normalized_grid,
            normalized_path,
            expected_start=expected_start,
            expected_goal=expected_goal,
        )
    except (PathPostprocessParameterError, RawPathError, TypeError, ValueError) as exc:
        return _failure_result(
            run_id=run_id,
            input_ref=input_ref,
            parameters=effective_parameters,
            code=(
                "INVALID_PARAMETERS"
                if isinstance(exc, PathPostprocessParameterError)
                else "INVALID_RAW_PATH"
            ),
            stage=(
                "validate_parameters"
                if isinstance(exc, PathPostprocessParameterError)
                else "validate_raw_path"
            ),
            error=exc,
            suggestion=(
                "check post-processing parameters and grid_size_m"
                if isinstance(exc, PathPostprocessParameterError)
                else "check planner output, map bounds, and obstacle cells"
            ),
        )

    fallback = False
    fallback_reason: str | None = None
    smoothed_curve: Sequence[Sequence[Real]] | None = None
    if not effective_parameters["smooth"]:
        effective_path = normalized_path
        smoothing_applied = False
        smoothing_method = "none"
    elif len(normalized_path) < 3:
        effective_path = normalized_path
        smoothing_applied = False
        smoothing_method = "natural_cubic_spline"
        fallback = True
        fallback_reason = "insufficient_points"
    else:
        active_smoother = smoother or NaturalCubicSplineSmoother()
        try:
            curve = active_smoother.smooth(
                normalized_path,
                effective_parameters["sample_spacing_grid"],
            )
            _validate_sampled_curve(
                normalized_grid,
                curve,
                start=normalized_path[0],
                goal=normalized_path[-1],
                sample_spacing_grid=effective_parameters["sample_spacing_grid"],
            )
            smoothed_curve = curve
            effective_path = _discretize_curve(
                curve,
                grid_shape=normalized_grid.shape,
                start=normalized_path[0],
                goal=normalized_path[-1],
            )
            smoothing_applied = True
            smoothing_method = type(active_smoother).__name__
        except (
            SmoothingError,
            RawPathError,
            ArithmeticError,
            TypeError,
            ValueError,
        ) as exc:
            effective_path = normalized_path
            smoothing_applied = False
            smoothing_method = type(active_smoother).__name__
            fallback = True
            if isinstance(exc, (SmoothingError, RawPathError)):
                message = str(exc).lower()
                if "blocked" in message or "obstacle" in message:
                    fallback_reason = "collision"
                elif "out of bounds" in message:
                    fallback_reason = "out_of_bounds"
                else:
                    fallback_reason = "numeric_error"
            else:
                fallback_reason = "numeric_error"
        except Exception as exc:
            return _failure_result(
                run_id=run_id,
                input_ref=input_ref,
                parameters=effective_parameters,
                code="SMOOTHER_ERROR",
                stage="smooth",
                error=exc,
                suggestion="检查可替换平滑器实现并在新运行批次中重试",
            )

    try:
        _validate_path(normalized_grid, effective_path)
        segments = _make_segments(
            effective_path,
            grid_size_m=float(grid_size_m),
            max_segment_length_grid=effective_parameters["max_segment_length_grid"],
            move_speed_m_s=effective_parameters["move_speed_m_s"],
            turn_speed_rad_s=effective_parameters["turn_speed_rad_s"],
            initial_heading=effective_parameters["initial_heading"],
        )
    except (RawPathError, SegmentationError, ValueError, TypeError) as exc:
        return _failure_result(
            run_id=run_id,
            input_ref=input_ref,
            parameters=effective_parameters,
            code="POSTPROCESSING_ERROR",
            stage="segment",
            error=exc,
            suggestion="reduce path step size or increase max_segment_length_grid",
        )

    smoothing_output = {
        "method": smoothing_method,
        "applied": smoothing_applied,
        "fallback": fallback,
        "sample_spacing_grid": effective_parameters["sample_spacing_grid"],
    }
    if fallback_reason:
        smoothing_output["fallback_reason"] = fallback_reason
    metric_path: Sequence[Sequence[Real]] = (
        smoothed_curve if smoothing_applied and smoothed_curve is not None
        else effective_path
    )
    metrics = metric_payload(
        {
            "raw_node_count": len(normalized_path),
            "raw_path_length_grid": _path_length(normalized_path),
            "raw_path_length_m": _path_length(normalized_path) * float(grid_size_m),
            "smoothed_node_count": len(metric_path),
            "smoothed_path_length_grid": _path_length(metric_path),
            "smoothed_path_length_m": _path_length(metric_path) * float(grid_size_m),
            "segment_count": len(segments),
            "estimated_time_s": sum(item["estimated_time_s"] for item in segments),
            "smoothing_fallback": fallback,
        },
        {
            "raw_node_count": "count",
            "raw_path_length_grid": "grid_cell",
            "raw_path_length_m": "m",
            "smoothed_node_count": "count",
            "smoothed_path_length_grid": "grid_cell",
            "smoothed_path_length_m": "m",
            "segment_count": "count",
            "estimated_time_s": "s",
            "smoothing_fallback": "boolean",
        },
    )
    return make_result(
        MODULE,
        run_id,
        STATUS_SUCCESS,
        inputs={
            "planner_result": os.path.abspath(input_ref) if input_ref else "<memory>",
            "start": {"row": normalized_path[0][0], "col": normalized_path[0][1]},
            "goal": {"row": normalized_path[-1][0], "col": normalized_path[-1][1]},
        },
        outputs={
            "raw_path": path_payload(normalized_path),
            "smoothed_path": (
                path_payload(smoothed_curve, dtype="float64")
                if smoothing_applied and smoothed_curve is not None
                else path_payload(effective_path)
            ),
            "segments": segments,
            "smoothing": smoothing_output,
            "metrics": metrics,
        },
        parameters={
            **effective_parameters,
            "grid_size_m": float(grid_size_m),
        },
    )


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
    "DEFAULT_PARAMETERS",
    "MODULE",
    "PathPostprocessParameterError",
    "process_path",
    "resolve_postprocess_parameters",
]
