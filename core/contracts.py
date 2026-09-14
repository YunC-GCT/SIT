"""跨模块 JSON 契约。

所有模块都返回同一外层结构；地图、路径、路径段、指标和模块清单使用
显式 JSON 结构表达单位、坐标含义、维度和数据类型。写入采用临时文件
替换，避免失败结果截断或覆盖已有成功结果。
"""

from __future__ import annotations

import argparse
import json
import os
import platform
import tempfile
from datetime import datetime, timezone
from typing import Any

SCHEMA_VERSION = "1.0"
STATUS_PENDING = "pending"
STATUS_RUNNING = "running"
STATUS_SUCCESS = "success"
STATUS_FAILED = "failed"
STATUSES = {STATUS_PENDING, STATUS_RUNNING, STATUS_SUCCESS, STATUS_FAILED}
ROW_COL_COORDINATE_SYSTEM = {
    "axes": ["row", "col"],
    "row": "increases_south",
    "col": "increases_east",
    "unit": "grid_cell",
}


class ContractError(ValueError):
    """输入或输出不符合统一契约。"""


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def make_error(
    code: str,
    message: str | None = None,
    *,
    module: str | None = None,
    stage: str | None = None,
    error_type: str | None = None,
    reason: str | None = None,
    input_ref: str | None = None,
    suggestion: str | None = None,
    details: Any = None,
) -> dict:
    """Create a structured module error.

    ``message`` is kept for compatibility with earlier callers; new callers
    should fill ``reason`` and ``input_ref`` so downstream recovery can point at
    the failing stage and input.
    """
    effective_reason = reason or message
    error = {"code": code}
    if message:
        error["message"] = message
    if module:
        error["module"] = module
    if stage:
        error["stage"] = stage
    if error_type:
        error["error_type"] = error_type
    if effective_reason:
        error["reason"] = effective_reason
    if input_ref:
        error["input"] = input_ref
    if suggestion:
        error["suggestion"] = suggestion
    if details is not None:
        error["details"] = details
    return error


def _json_ready(value: Any) -> Any:
    if hasattr(value, "tolist"):
        return value.tolist()
    if isinstance(value, tuple):
        return [_json_ready(item) for item in value]
    if isinstance(value, list):
        return [_json_ready(item) for item in value]
    if isinstance(value, dict):
        return {key: _json_ready(item) for key, item in value.items()}
    return value


def _shape_of(value: Any) -> list[int]:
    if hasattr(value, "shape"):
        return [int(part) for part in value.shape]
    if isinstance(value, (list, tuple)):
        length = len(value)
        if length == 0:
            return [0]
        child_shape = _shape_of(value[0])
        for item in value[1:]:
            if _shape_of(item) != child_shape:
                raise ContractError("数组必须是规则矩形结构")
        return [length] + child_shape
    return []


def _point(row_col: tuple[int, int] | list[int]) -> dict:
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
    points: list[tuple[int, int]] | list[list[int]] | list[tuple[float, float]] | list[list[float]],
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
    start: tuple[int, int] | list[int],
    end: tuple[int, int] | list[int],
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


def make_result(module: str, run_id: str, status: str, *, inputs=None,
                outputs=None, parameters=None, errors=None, logs=None,
                metadata=None, message: str | None = None) -> dict:
    if status not in STATUSES:
        raise ContractError(f"未知结果状态: {status}")
    errors = [_error_with_default_module(error, module) for error in (errors or [])]
    if status == STATUS_FAILED and not errors:
        raise ContractError("failed 结果必须包含结构化 errors")
    result = {
        "schema_version": SCHEMA_VERSION,
        "status": status,
        "module": module,
        "run_id": run_id,
        "created_at": utc_now(),
        "inputs": inputs or {},
        "outputs": outputs or {},
        "parameters": parameters or {},
        "logs": logs or [],
        "errors": errors,
        "metadata": metadata or {},
    }
    if message:
        result["message"] = message
    return validate_result(result)


def validate_result(result: dict) -> dict:
    if not isinstance(result, dict):
        raise ContractError("结果必须是 JSON 对象")
    required = {"schema_version", "status", "module", "run_id", "created_at",
                "inputs", "outputs", "parameters", "logs", "errors"}
    missing = sorted(required - set(result))
    if missing:
        raise ContractError(f"结果缺少字段: {', '.join(missing)}")
    if result["schema_version"] != SCHEMA_VERSION:
        raise ContractError(f"不支持的 schema_version: {result['schema_version']}")
    if result["status"] not in STATUSES:
        raise ContractError(f"未知结果状态: {result['status']}")
    if not isinstance(result["errors"], list):
        raise ContractError("errors 必须是数组")
    if not isinstance(result["logs"], list):
        raise ContractError("logs 必须是数组")
    if result["status"] == STATUS_FAILED and not result["errors"]:
        raise ContractError("failed 结果必须包含结构化 errors")
    for error in result["errors"]:
        validate_error(error)
    _validate_known_outputs(result.get("outputs", {}))
    return result


def _error_with_default_module(error: Any, module: str) -> Any:
    if isinstance(error, dict) and "module" not in error:
        return {**error, "module": module}
    return error


def validate_error(error: Any) -> dict:
    """Validate a structured error and return it."""
    if not isinstance(error, dict):
        raise ContractError("error 必须是对象")
    required = {"code", "module", "stage", "reason", "input", "suggestion"}
    missing = sorted(required - set(error))
    if missing:
        raise ContractError(f"结构化错误缺少字段: {', '.join(missing)}")
    for key in required:
        if not isinstance(error[key], str) or not error[key]:
            raise ContractError(f"结构化错误字段 {key} 必须是非空字符串")
    return error


def _validate_known_outputs(outputs: Any) -> None:
    if not isinstance(outputs, dict):
        raise ContractError("outputs 必须是对象")
    if "map" in outputs:
        _validate_map_payload(outputs["map"])
    for key in ("path", "raw_path", "smoothed_path"):
        if key in outputs:
            _validate_path_payload(outputs[key], key)
    if "segments" in outputs:
        _validate_segments_payload(outputs["segments"])
    if "metrics" in outputs:
        _validate_metrics_payload(outputs["metrics"])
    if "modules" in outputs:
        _validate_modules_payload(outputs["modules"])


def _validate_array_payload(payload: Any, name: str) -> None:
    if not isinstance(payload, dict):
        raise ContractError(f"{name} 必须是对象")
    required = {"values", "shape", "dtype", "units", "coordinate_system"}
    missing = sorted(required - set(payload))
    if missing:
        raise ContractError(f"{name} 缺少字段: {', '.join(missing)}")
    if not isinstance(payload["shape"], list) or not all(isinstance(item, int) for item in payload["shape"]):
        raise ContractError(f"{name}.shape 必须是整数数组")
    actual_shape = _shape_of(payload["values"])
    empty_matrix_shape = (
        isinstance(payload["values"], list)
        and not payload["values"]
        and payload["shape"] == [0, 2]
    )
    if payload["shape"] != actual_shape and not empty_matrix_shape:
        raise ContractError(f"{name}.shape 与 values 实际维度不一致")
    if not isinstance(payload["dtype"], str) or not payload["dtype"]:
        raise ContractError(f"{name}.dtype 必须是非空字符串")
    if not isinstance(payload["units"], dict):
        raise ContractError(f"{name}.units 必须是对象")
    if not isinstance(payload["coordinate_system"], dict):
        raise ContractError(f"{name}.coordinate_system 必须是对象")


def _validate_row_col_coordinate_system(payload: dict, name: str) -> None:
    axes = payload.get("coordinate_system", {}).get("axes")
    if axes != ["row", "col"]:
        raise ContractError(f"{name} 坐标轴必须是 row/col")


def _validate_map_payload(payload: Any) -> None:
    if not isinstance(payload, dict):
        raise ContractError("map 必须是对象")
    required = {"grid", "grid_size_m", "free_value", "obstacle_value", "coordinate_system"}
    missing = sorted(required - set(payload))
    if missing:
        raise ContractError(f"map 缺少字段: {', '.join(missing)}")
    _validate_array_payload(payload["grid"], "map.grid")
    _validate_row_col_coordinate_system(payload["grid"], "map.grid")
    _validate_row_col_coordinate_system(payload, "map")
    if len(payload["grid"]["shape"]) != 2:
        raise ContractError("map.grid.shape 必须是二维")
    if not isinstance(payload["grid_size_m"], (int, float)) or payload["grid_size_m"] <= 0:
        raise ContractError("map.grid_size_m 必须是正数")


def _validate_path_payload(payload: Any, name: str) -> None:
    if not isinstance(payload, dict) or "points" not in payload or "coordinate_system" not in payload:
        raise ContractError(f"{name} 缺少 points 或 coordinate_system")
    _validate_array_payload(payload["points"], f"{name}.points")
    _validate_row_col_coordinate_system(payload["points"], f"{name}.points")
    _validate_row_col_coordinate_system(payload, name)
    shape = payload["points"]["shape"]
    if len(shape) != 2 or shape[1] != 2:
        raise ContractError(f"{name}.points.shape 必须是 [N, 2]")


def _validate_segments_payload(payload: Any) -> None:
    if not isinstance(payload, list):
        raise ContractError("segments 必须是数组")
    required = {
        "index",
        "start",
        "end",
        "delta",
        "direction_deg",
        "turn_angle_deg",
        "grid_distance",
        "distance_m",
        "estimated_time_s",
        "motion_type",
        "units",
        "coordinate_system",
    }
    required_units = {"direction_deg", "turn_angle_deg", "grid_distance", "distance_m", "estimated_time_s"}
    for segment in payload:
        if not isinstance(segment, dict):
            raise ContractError("segment 必须是对象")
        missing = sorted(required - set(segment))
        if missing:
            raise ContractError(f"segment 缺少字段: {', '.join(missing)}")
        _validate_row_col_coordinate_system(segment, "segment")
        if not isinstance(segment["index"], int):
            raise ContractError("segment.index 必须是整数")
        for point_name in ("start", "end"):
            point = segment[point_name]
            if not isinstance(point, dict) or set(point) != {"row", "col"}:
                raise ContractError(f"segment.{point_name} 必须包含 row/col")
            if not isinstance(point["row"], int) or not isinstance(point["col"], int):
                raise ContractError(f"segment.{point_name} row/col 必须是整数")
        delta = segment["delta"]
        if not isinstance(delta, dict) or set(delta) != {"d_row", "d_col"}:
            raise ContractError("segment.delta 必须包含 d_row/d_col")
        if not isinstance(delta["d_row"], int) or not isinstance(delta["d_col"], int):
            raise ContractError("segment.delta d_row/d_col 必须是整数")
        expected_delta = {
            "d_row": segment["end"]["row"] - segment["start"]["row"],
            "d_col": segment["end"]["col"] - segment["start"]["col"],
        }
        if delta != expected_delta:
            raise ContractError("segment.delta 与 start/end 不一致")
        for field in ("direction_deg", "turn_angle_deg", "grid_distance", "distance_m", "estimated_time_s"):
            if not isinstance(segment[field], (int, float)):
                raise ContractError(f"segment.{field} 必须是数字")
        if segment["grid_distance"] < 0 or segment["distance_m"] < 0 or segment["estimated_time_s"] < 0:
            raise ContractError("segment 距离和时间必须非负")
        if not isinstance(segment["motion_type"], str) or not segment["motion_type"]:
            raise ContractError("segment.motion_type 必须是非空字符串")
        if not isinstance(segment["units"], dict) or not required_units.issubset(segment["units"]):
            raise ContractError("segment.units 缺少必需单位")


def _validate_metrics_payload(payload: Any) -> None:
    if not isinstance(payload, dict) or not isinstance(payload.get("values"), dict) or not isinstance(payload.get("units"), dict):
        raise ContractError("metrics 必须包含 values 和 units 对象")
    missing_units = sorted(set(payload["values"]) - set(payload["units"]))
    if missing_units:
        raise ContractError(f"metrics 缺少单位: {', '.join(missing_units)}")


def _validate_modules_payload(payload: Any) -> None:
    if not isinstance(payload, list):
        raise ContractError("modules 必须是数组")
    required = {"name", "status", "updated_at", "version", "inputs", "outputs", "parameters"}
    for module in payload:
        if not isinstance(module, dict):
            raise ContractError("module entry 必须是对象")
        missing = sorted(required - set(module))
        if missing:
            raise ContractError(f"module entry 缺少字段: {', '.join(missing)}")
        if module["status"] not in STATUSES:
            raise ContractError(f"未知模块状态: {module['status']}")


def write_result(path: str, result: dict, *, overwrite: bool = True) -> str:
    """校验并原子写入结果；失败结果不会覆盖已有成功结果。"""
    result = validate_result(result)
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    if os.path.exists(path) and not overwrite:
        raise FileExistsError(path)
    if os.path.exists(path):
        try:
            existing = read_result(path)
        except (json.JSONDecodeError, OSError):
            # 损坏的旧文件可以被新的结果替换；契约校验会阻止有效文件被覆盖。
            pass
        else:
            if existing.get("status") == STATUS_SUCCESS and result.get("status") != STATUS_SUCCESS:
                raise ContractError("失败结果不能覆盖已有 success 结果")
    directory = os.path.dirname(os.path.abspath(path))
    fd, tmp = tempfile.mkstemp(prefix=".result-", suffix=".json", dir=directory)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            json.dump(result, handle, ensure_ascii=False, indent=2)
            handle.write("\n")
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)
    return path


def read_result(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as handle:
        return validate_result(json.load(handle))


def require_success(path: str, *, module: str | None = None) -> dict:
    result = read_result(path)
    if result["status"] != STATUS_SUCCESS:
        raise ContractError(f"下游拒绝非成功结果: {path} ({result['status']})")
    if module and result["module"] != module:
        raise ContractError(f"结果模块不匹配: 期望 {module}, 实际 {result['module']}")
    return result


def _main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="统一 JSON 契约读写与校验工具")
    subparsers = parser.add_subparsers(dest="command", required=True)

    validate_parser = subparsers.add_parser("validate", help="校验结果 JSON")
    validate_parser.add_argument("path")
    validate_parser.add_argument("--require-success", action="store_true")
    validate_parser.add_argument("--module")

    args = parser.parse_args(argv)
    if args.command == "validate":
        if args.require_success:
            result = require_success(args.path, module=args.module)
        else:
            result = read_result(args.path)
            if args.module and result["module"] != args.module:
                raise ContractError(f"结果模块不匹配: 期望 {args.module}, 实际 {result['module']}")
        print(json.dumps({"status": "valid", "module": result["module"], "result_status": result["status"]}, ensure_ascii=False))
        return 0

    raise ContractError(f"未知命令: {args.command}")


if __name__ == "__main__":
    raise SystemExit(_main())
