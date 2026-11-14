"""规划结果载荷、指标及结构化失败信息。"""

from __future__ import annotations

import math
import os

from core import (
    STATUS_FAILED,
    make_error,
    make_result,
    metric_payload,
    path_payload,
)

MODULE = "path_planner"
METRIC_UNITS = {
    "found_path": "boolean",
    "planning_time_s": "s",
    "raw_node_count": "count",
    "raw_path_length_grid": "grid_cell",
    "raw_path_length_m": "m",
    "random_seed": "seed",
}

def _path_length_grid(path: list[tuple[int, int]]) -> float:
    """累加路径相邻点的欧氏距离，单位为栅格。"""
    return sum(
        math.hypot(next_point[0] - point[0], next_point[1] - point[1])
        for point, next_point in zip(path, path[1:])
    )

def _metrics(
    path: list[tuple[int, int]],
    planning_time_s: float,
    grid_size_m: float,
    seed: int | None,
) -> dict:
    """封装规划耗时、点数和栅格/米制长度等指标及其单位。"""
    length_grid = _path_length_grid(path)
    return metric_payload(
        {
            "found_path": bool(path),
            "planning_time_s": float(planning_time_s),
            "raw_node_count": len(path),
            "raw_path_length_grid": length_grid,
            "raw_path_length_m": length_grid * float(grid_size_m),
            "random_seed": seed,
        },
        METRIC_UNITS,
    )

def _inputs(input_ref: str, start: tuple[int, int], goal: tuple[int, int]) -> dict:
    """构造规划输入引用，记录地图来源与起终点。"""
    return {
        "map_result": os.path.abspath(input_ref) if input_ref else "<memory-grid>",
        "start": {"row": start[0], "col": start[1]},
        "goal": {"row": goal[0], "col": goal[1]},
    }

def _failure(
    *,
    run_id: str,
    input_ref: str,
    start: tuple[int, int] | None,
    goal: tuple[int, int] | None,
    algorithm: str,
    parameters: dict,
    seed: int | None,
    grid_size_m: float,
    code: str,
    stage: str,
    error_type: str,
    reason: str,
    suggestion: str,
    planning_time_s: float = 0.0,
) -> dict:
    """构造算法失败的标准结果，保留参数、输入及结构化故障信息。"""
    safe_start = start or (0, 0)
    safe_goal = goal or (0, 0)
    try:
        safe_grid_size = float(grid_size_m)
    except (TypeError, ValueError):
        safe_grid_size = 1.0
    return make_result(
        MODULE,
        run_id,
        STATUS_FAILED,
        inputs=_inputs(input_ref, safe_start, safe_goal),
        outputs={
            "raw_path": path_payload([]),
            "metrics": _metrics([], planning_time_s, safe_grid_size, seed),
        },
        parameters={
            "algorithm": algorithm,
            "algorithm_parameters": parameters,
            "seed": seed,
            "grid_size_m": grid_size_m,
        },
        errors=[
            make_error(
                code,
                module=MODULE,
                stage=stage,
                error_type=error_type,
                reason=reason,
                input_ref=input_ref or "<memory-grid>",
                suggestion=suggestion,
            )
        ],
    )


__all__ = [
    "METRIC_UNITS",
    "MODULE",
    "_failure",
    "_inputs",
    "_metrics",
    "_path_length_grid",
]
