"""Canonical JSON payload builders for map, path, segments, and metrics."""

from __future__ import annotations

import platform
from typing import Any, Sequence

from .contracts_types import (
    ContractError,
    ROW_COL_COORDINATE_SYSTEM,
    STATUSES,
    _json_ready,
    _shape_of,
    utc_now,
)

def _point(row_col: Sequence[float]) -> dict:
    if len(row_col) != 2:
        raise ContractError("坐标必须包含 row 和 col")
    row, col = row_col
    return {"row": int(row), "col": int(col)}

def array_payload(
    values: Any,
    *,
    dtype: str,
    units: dict[str, str] | None = None,
    coordinate_system: dict[str, Any] | None = None,
) -> dict:
    """Represent an array-like value with explicit JSON metadata."""
    json_values = _json_ready(values)
    return {
        "values": json_values,
        "shape": _shape_of(json_values),
        "dtype": dtype,
        "units": units or {},
        "coordinate_system": coordinate_system or {},
    }

def grid_map_payload(
    grid: Any,
    *,
    grid_size_m: float,
    start: tuple[int, int] | list[int] | None = None,
    goal: tuple[int, int] | list[int] | None = None,
    free_value: int = 0,
    obstacle_value: int = 1,
    dtype: str = "uint8",
) -> dict:
    """Build the canonical grid-map JSON output."""
    payload = {
        "grid": array_payload(
            grid,
            dtype=dtype,
            units={"values": "occupancy", "grid_size": "m/cell"},
            coordinate_system=ROW_COL_COORDINATE_SYSTEM,
        ),
        "grid_size_m": float(grid_size_m),
        "free_value": int(free_value),
        "obstacle_value": int(obstacle_value),
        "coordinate_system": ROW_COL_COORDINATE_SYSTEM,
    }
    if start is not None:
        payload["start"] = _point(start)
    if goal is not None:
        payload["goal"] = _point(goal)
    return payload

def path_payload(
    points: Sequence[Sequence[float]],
    *,
    coordinate_system: dict[str, Any] | None = None,
    units: dict[str, str] | None = None,
    dtype: str = "int",
) -> dict:
    """Build a canonical row/col path payload."""
    if dtype.startswith("float"):
        normalized = [[float(row), float(col)] for row, col in points]
    else:
        normalized = [[int(row), int(col)] for row, col in points]
    points_payload = array_payload(
        normalized,
        dtype=dtype,
        units=units or {"coordinates": "grid_cell"},
        coordinate_system=coordinate_system or ROW_COL_COORDINATE_SYSTEM,
    )
    # 空路径仍然是一个 N×2 的坐标矩阵；普通 JSON 空数组无法保留第二维。
    if not normalized:
        points_payload["shape"] = [0, 2]
    return {
        "points": points_payload,
        "coordinate_system": coordinate_system or ROW_COL_COORDINATE_SYSTEM,
    }

def path_segment_payload(
    index: int,
    *,
    start: Sequence[float],
    end: Sequence[float],
    direction_deg: float,
    turn_angle_deg: float,
    grid_distance: float,
    distance_m: float,
    estimated_time_s: float,
    motion_type: str = "turn",
) -> dict:
    """Build one executable path segment for motion-control input."""
    start_point = _point(start)
    end_point = _point(end)
    return {
        "index": int(index),
        "start": start_point,
        "end": end_point,
        "delta": {
            "d_row": end_point["row"] - start_point["row"],
            "d_col": end_point["col"] - start_point["col"],
        },
        "direction_deg": float(direction_deg),
        "turn_angle_deg": float(turn_angle_deg),
        "grid_distance": float(grid_distance),
        "distance_m": float(distance_m),
        "estimated_time_s": float(estimated_time_s),
        "motion_type": motion_type,
        "units": {
            "direction_deg": "deg",
            "turn_angle_deg": "deg",
            "grid_distance": "grid_cell",
            "distance_m": "m",
            "estimated_time_s": "s",
        },
        "coordinate_system": ROW_COL_COORDINATE_SYSTEM,
    }

def metric_payload(values: dict[str, Any], units: dict[str, str]) -> dict:
    """Build canonical planning or execution metrics."""
    return {"values": _json_ready(values), "units": units}

def module_entry(
    name: str,
    status: str,
    *,
    result_path: str | None = None,
    updated_at: str | None = None,
    version: dict[str, Any] | None = None,
    inputs: dict[str, Any] | None = None,
    outputs: dict[str, Any] | None = None,
    parameters: dict[str, Any] | None = None,
) -> dict:
    """Build one module-list entry for a run manifest or result."""
    if status not in STATUSES:
        raise ContractError(f"未知模块状态: {status}")
    entry = {
        "name": name,
        "status": status,
        "updated_at": updated_at or utc_now(),
        "version": version or {"python": platform.python_version()},
        "inputs": inputs or {},
        "outputs": outputs or {},
        "parameters": parameters or {},
    }
    if result_path:
        entry["result"] = result_path
    return entry


__all__ = [
    "array_payload",
    "grid_map_payload",
    "metric_payload",
    "module_entry",
    "path_payload",
    "path_segment_payload",
]
