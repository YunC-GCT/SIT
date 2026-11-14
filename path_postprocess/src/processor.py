"""路径后处理主流程与结果构造。"""

from __future__ import annotations

import os
from typing import Any, Iterable, Sequence

from core import (
    STATUS_FAILED,
    STATUS_SUCCESS,
    make_error,
    make_result,
    metric_payload,
    path_payload,
)

from .geometry import (
    RawPathError,
    _normalize_endpoint,
    _normalize_grid,
    _normalize_path,
    _path_length,
    _validate_path,
)
from .parameters import (
    PathPostprocessParameterError,
    resolve_postprocess_parameters,
)
from .segments import (
    SegmentationError,
    _discretize_curve,
    _make_segments,
    _validate_sampled_curve,
)
from .smoothers import NaturalCubicSplineSmoother, PathSmoother, SmoothingError

MODULE = "path_postprocess"

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
    """构造后处理失败结果，包含失败阶段、输入、参数及恢复建议。"""
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
    """校验规划路径，按配置平滑、碰撞检查并生成可执行路径段及统一结果。

    平滑器通过 PathSmoother 接口替换；平滑失败或曲线穿障时回退到已校验的原路径。"""
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
    smoothed_curve: Sequence[Sequence[float]] | None = None
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
        segments, normalization_fallback = _make_segments(
            normalized_grid,
            effective_path,
            curve=(
                smoothed_curve
                if smoothing_applied and smoothed_curve is not None
                else None
            ),
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
    metric_path: Sequence[Sequence[float]] = (
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
            "segment_normalization_fallback": normalization_fallback,
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
            "segment_normalization_fallback": "boolean",
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


__all__ = [
    "MODULE",
    "process_path",
]
