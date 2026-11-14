"""Planner execution for single and research modes."""

from __future__ import annotations

import os
import time
from typing import Any, Iterable

from core import (
    STATUS_FAILED,
    STATUS_SUCCESS,
    make_result,
    path_payload,
)

from .planner_factory import (
    RANDOM_ALGORITHMS,
    PlannerParameterError,
    create_planner,
    resolve_algorithm_parameters,
)
from .planner_results import MODULE, _failure, _inputs, _metrics
from .planner_validation import _normalize_path, _validate_inputs

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
        failed_result = _failure(
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
            error_type="search_budget_exhausted" if algorithm in RANDOM_ALGORITHMS else "unreachable",
            reason=(f"{algorithm} 在本次采样/迭代预算内未找到从 {safe_start} 到 {safe_goal} 的可行路径，不能据此判定地图不连通"
                    if algorithm in RANDOM_ALGORITHMS else
                    f"{algorithm} 未找到从 {safe_start} 到 {safe_goal} 的可行路径"),
            suggestion="检查起终点连通性；采样算法可增加采样/迭代预算、减小扩展步长并比较多个随机种子",
            planning_time_s=elapsed,
        )
        failed_result["outputs"]["search_diagnostics"] = getattr(planner, "diagnostics", {})
        return failed_result

    return make_result(
        MODULE,
        run_id,
        STATUS_SUCCESS,
        inputs=_inputs(input_ref, safe_start, safe_goal),
        outputs={
            "search_diagnostics": getattr(planner, "diagnostics", {}),
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
