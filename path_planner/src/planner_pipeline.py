"""统一规划器运行接口。

该模块把五种规划器的内部 ``plan`` 方法收敛为一个可独立测试的接口：
输入是栅格地图、row/col 起终点、算法参数和随机种子，输出是统一 JSON
结果中的原始路径与实验指标。
"""

from __future__ import annotations

import math
import os
import time
from numbers import Integral, Real
from typing import Any, Iterable

import numpy as np

from core import (
    STATUS_FAILED,
    STATUS_SUCCESS,
    make_error,
    make_result,
    metric_payload,
    path_payload,
)

from .planner_factory import (
    PlannerParameterError,
    create_planner,
    resolve_algorithm_parameters,
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


def _path_length_grid(path: list[tuple[int, int]]) -> float:
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


def run_algorithm(
    grid: Any,
    start: Iterable[int],
    goal: Iterable[int],
    algorithm: str,
    *,
    parameters: dict | None = None,
    seed: int | None = None,
    grid_size_m: float = 0.3,
    run_id: str = "standalone",
    input_ref: str = "",
) -> dict:
    """Run one registered planner and return a validated contract result."""
    raw_parameters = parameters if isinstance(parameters, dict) else {}
    safe_start = safe_goal = None
    try:
        if parameters is not None and not isinstance(parameters, dict):
            raise PlannerParameterError("算法参数必须是 JSON 对象")
        raw_parameters = dict(parameters or {})
        if "seed" in raw_parameters:
            if seed is not None:
                raise PlannerParameterError(
                    "seed 只能通过 seed 参数或 parameters.seed 提供一次"
                )
            seed = raw_parameters.pop("seed")
        normalized_grid, safe_start, safe_goal = _validate_inputs(
            grid, start, goal, grid_size_m
        )
        raw_parameters = resolve_algorithm_parameters(algorithm, raw_parameters)
        planner = create_planner(
            algorithm,
            normalized_grid,
            params=raw_parameters,
            seed=seed,
        )
    except (PlannerParameterError, ValueError, TypeError) as exc:
        return _failure(
            run_id=run_id,
            input_ref=input_ref,
            start=safe_start,
            goal=safe_goal,
            algorithm=algorithm,
            parameters=raw_parameters,
            seed=seed,
            grid_size_m=grid_size_m,
            code="INVALID_PARAMETERS",
            stage="validate",
            error_type=type(exc).__name__,
            reason=str(exc),
            suggestion="检查算法名称、起终点、grid 和算法参数后重试",
        )

    started = time.perf_counter()
    try:
        path = planner.plan(safe_start, safe_goal)
        elapsed = time.perf_counter() - started
        normalized_path = _normalize_path(path, normalized_grid, safe_start, safe_goal)
    except Exception as exc:
        elapsed = time.perf_counter() - started
        return _failure(
            run_id=run_id,
            input_ref=input_ref,
            start=safe_start,
            goal=safe_goal,
            algorithm=algorithm,
            parameters=raw_parameters,
            seed=seed,
            grid_size_m=float(grid_size_m),
            code="PLANNING_ERROR",
            stage="plan",
            error_type=type(exc).__name__,
            reason=str(exc),
            suggestion="检查地图和算法参数；保留当前运行批次后重试",
            planning_time_s=elapsed,
        )

    if not normalized_path:
        return _failure(
            run_id=run_id,
            input_ref=input_ref,
            start=safe_start,
            goal=safe_goal,
            algorithm=algorithm,
            parameters=raw_parameters,
            seed=seed,
            grid_size_m=float(grid_size_m),
            code="NO_PATH",
            stage="plan",
            error_type="unreachable",
            reason=f"{algorithm} 未找到从 {safe_start} 到 {safe_goal} 的可行路径",
            suggestion="检查障碍栅格、起终点连通性或增加采样/迭代参数",
            planning_time_s=elapsed,
        )

    return make_result(
        MODULE,
        run_id,
        STATUS_SUCCESS,
        inputs=_inputs(input_ref, safe_start, safe_goal),
        outputs={
            "raw_path": path_payload(normalized_path),
            "metrics": _metrics(
                normalized_path, elapsed, float(grid_size_m), seed
            ),
        },
        parameters={
            "algorithm": algorithm,
            "algorithm_parameters": raw_parameters,
            "seed": seed,
            "grid_size_m": float(grid_size_m),
        },
    )


def _comparison_entry(result: dict) -> dict:
    return {
        "algorithm": result["parameters"]["algorithm"],
        "status": result["status"],
        "raw_path": result["outputs"]["raw_path"],
        "metrics": result["outputs"]["metrics"],
        "parameters": result["parameters"]["algorithm_parameters"],
        "seed": result["parameters"]["seed"],
        "errors": result["errors"],
    }


def run_planning(
    grid: Any,
    start: Iterable[int],
    goal: Iterable[int],
    algorithms: Iterable[str],
    *,
    parameters: dict[str, dict] | None = None,
    seed: int | None = None,
    grid_size_m: float = 0.3,
    run_id: str = "standalone",
    input_ref: str = "",
    mode: str = "research",
) -> dict:
    """Run one or more planners and return a comparable experiment result."""
    try:
        selected = list(algorithms)
    except TypeError as exc:
        return _failure(
            run_id=run_id,
            input_ref=input_ref,
            start=None,
            goal=None,
            algorithm="<multiple>",
            parameters={},
            seed=seed,
            grid_size_m=grid_size_m,
            code="INVALID_PARAMETERS",
            stage="validate",
            error_type=type(exc).__name__,
            reason="algorithms 必须是算法名称数组",
            suggestion="提供一个或多个已注册算法名称",
        )
    if not selected:
        return _failure(
            run_id=run_id,
            input_ref=input_ref,
            start=None,
            goal=None,
            algorithm="<none>",
            parameters={},
            seed=seed,
            grid_size_m=grid_size_m,
            code="INVALID_PARAMETERS",
            stage="validate",
            error_type="ValueError",
            reason="至少需要选择一个规划算法",
            suggestion="提供一个或多个已注册算法名称",
        )
    if mode not in {"single", "research"}:
        return _failure(
            run_id=run_id,
            input_ref=input_ref,
            start=None,
            goal=None,
            algorithm="<multiple>",
            parameters={},
            seed=seed,
            grid_size_m=grid_size_m,
            code="INVALID_PARAMETERS",
            stage="validate",
            error_type="ValueError",
            reason="mode 必须是 single 或 research",
            suggestion="使用 mode=single 或 mode=research",
        )

    try:
        normalized_grid, normalized_start, normalized_goal = _validate_inputs(
            grid, start, goal, grid_size_m
        )
    except (PlannerParameterError, ValueError, TypeError) as exc:
        return _failure(
            run_id=run_id,
            input_ref=input_ref,
            start=None,
            goal=None,
            algorithm="<multiple>",
            parameters=parameters or {},
            seed=seed,
            grid_size_m=grid_size_m,
            code="INVALID_PARAMETERS",
            stage="validate",
            error_type=type(exc).__name__,
            reason=str(exc),
            suggestion="检查地图、起终点和 grid_size_m 后重试",
        )

    if parameters is not None and not isinstance(parameters, dict):
        return _failure(
            run_id=run_id,
            input_ref=input_ref,
            start=normalized_start,
            goal=normalized_goal,
            algorithm="<multiple>",
            parameters={},
            seed=seed,
            grid_size_m=grid_size_m,
            code="INVALID_PARAMETERS",
            stage="validate",
            error_type="PlannerParameterError",
            reason="parameters 必须是按算法名称分组的 JSON 对象",
            suggestion="使用 {'astar': {...}, 'rrt': {...}} 形式提供参数",
        )
    per_algorithm = parameters or {}
    results = [
        run_algorithm(
            normalized_grid,
            normalized_start,
            normalized_goal,
            algorithm,
            parameters=per_algorithm.get(algorithm),
            seed=seed,
            grid_size_m=grid_size_m,
            run_id=run_id,
            input_ref=input_ref,
        )
        for algorithm in selected
    ]
    if mode == "single" and len(results) == 1:
        return results[0]

    comparisons = [_comparison_entry(result) for result in results]
    errors = [
        error
        for result in results
        for error in result["errors"]
    ]
    status = STATUS_FAILED if errors else STATUS_SUCCESS
    return make_result(
        MODULE,
        run_id,
        status,
        inputs={
            "map_result": os.path.abspath(input_ref) if input_ref else "<memory-grid>",
            "start": {"row": normalized_start[0], "col": normalized_start[1]},
            "goal": {"row": normalized_goal[0], "col": normalized_goal[1]},
        },
        outputs={"comparisons": comparisons},
        parameters={
            "mode": mode,
            "algorithms": selected,
            "algorithm_parameters": {
                result["parameters"]["algorithm"]: result["parameters"]["algorithm_parameters"]
                for result in results
            },
            "seed": seed,
            "grid_size_m": float(grid_size_m),
        },
        errors=errors,
    )


__all__ = ["run_algorithm", "run_planning"]
