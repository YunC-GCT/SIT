#!/usr/bin/env python3
"""从地图与规划 JSON 结果运行路径后处理。"""

from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Sequence

import numpy as np
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from core import (  # noqa: E402
    RunBatch,
    STATUS_FAILED,
    STATUS_SUCCESS,
    make_error,
    make_result,
    path_payload,
    require_success,
    write_result,
)
from .src.path_postprocessor import (  # noqa: E402
    MODULE,
    process_path,
)


def _load_config(path: str) -> dict:
    """读取后处理 YAML 配置并返回字典。"""
    with open(path, encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}
    return raw.get("path_postprocess", raw)


def _read_map(path: str):
    """读取成功的地图结果并提取栅格、起终点及米制尺度。"""
    result = require_success(path, module="map_pipeline")
    payload = result["outputs"].get("map")
    if not isinstance(payload, dict):
        raise ValueError("map result is missing outputs.map")
    grid_payload = payload.get("grid", {})
    grid = np.asarray(grid_payload.get("values"))
    start_payload = payload.get("start")
    goal_payload = payload.get("goal")
    if not start_payload or not goal_payload:
        raise ValueError("map result must contain start and goal")
    start = (int(start_payload["row"]), int(start_payload["col"]))
    goal = (int(goal_payload["row"]), int(goal_payload["col"]))
    return grid, start, goal, float(payload["grid_size_m"])


def _read_planner(path: str):
    """读取成功的规划结果，并选取本次后处理使用的算法路径。"""
    result = require_success(path, module="path_planner")
    payload = result["outputs"].get("raw_path")
    if isinstance(payload, dict):
        points = payload.get("points", {}).get("values")
        if not isinstance(points, list):
            raise ValueError("planner result is missing outputs.raw_path.points.values")
        return [{"algorithm": result["parameters"].get("algorithm", "planner"), "points": points}]

    comparisons = result["outputs"].get("comparisons")
    if not isinstance(comparisons, list) or not comparisons:
        raise ValueError(
            "planner result must contain outputs.raw_path or outputs.comparisons"
        )
    entries = []
    for index, comparison in enumerate(comparisons):
        if not isinstance(comparison, dict):
            raise ValueError(f"planner comparison {index} must be an object")
        if comparison.get("status") != STATUS_SUCCESS:
            raise ValueError(
                f"planner comparison {index} is not successful: "
                f"{comparison.get('status')}"
            )
        comparison_path = comparison.get("raw_path", {})
        points = comparison_path.get("points", {}).get("values")
        if not isinstance(points, list):
            raise ValueError(
                f"planner comparison {index} is missing raw_path.points.values"
            )
        entries.append(
            {"algorithm": comparison.get("algorithm", f"planner-{index}"), "points": points}
        )
    return entries


def _failure_result(
    run_id: str,
    map_path: str,
    planner_path: str,
    stage: str,
    error: Exception,
    input_ref: str | None = None,
    code: str = "POSTPROCESS_INPUT_ERROR",
) -> dict:
    """将后处理 CLI 异常封装为结构化失败结果，保留输入和建议动作。"""
    return make_result(
        MODULE,
        run_id,
        STATUS_FAILED,
        inputs={
            "map_result": os.path.abspath(map_path),
            "planner_result": os.path.abspath(planner_path),
        },
        outputs={"raw_path": path_payload([])},
        errors=[
            make_error(
                code,
                module=MODULE,
                stage=stage,
                error_type=type(error).__name__,
                reason=str(error),
                input_ref=os.path.abspath(input_ref or planner_path),
                suggestion="check that map and planner JSON results are successful",
            )
        ],
    )


def main(argv: Sequence[str] | None = None) -> int:
    """解析后处理 CLI 参数，读取上游 JSON，生成平滑路径、执行段及预览并保存结果。"""
    parser = argparse.ArgumentParser(description="Generate executable path segments")
    parser.add_argument("--map-result", required=True)
    parser.add_argument("--planner-result", required=True)
    parser.add_argument("--params", help="post-processing parameter JSON object")
    parser.add_argument("--output")
    parser.add_argument("--run-root", default="runs")
    parser.add_argument("--run-id")
    parser.add_argument(
        "--config",
        default=os.path.join(os.path.dirname(__file__), "config.yaml"),
    )
    args = parser.parse_args(argv)

    batch = RunBatch(root=args.run_root, run_id=args.run_id).start()
    batch.register_module(MODULE)
    output_path = os.path.abspath(args.output) if args.output else os.path.join(
        batch.stage_dir(MODULE), "result.json"
    )

    try:
        config = _load_config(args.config)
        parameters = dict(config)
        if args.params:
            override = json.loads(args.params)
            if not isinstance(override, dict):
                raise ValueError("--params must be a JSON object")
            parameters.update(override)
    except (
        OSError,
        ValueError,
        TypeError,
        KeyError,
        AttributeError,
        json.JSONDecodeError,
        yaml.YAMLError,
    ) as exc:
        result = _failure_result(
            batch.run_id,
            args.map_result,
            args.planner_result,
            "load_config",
            exc,
            input_ref=args.config,
            code="POSTPROCESS_CONFIG_ERROR",
        )
    else:
        try:
            grid, start, goal, grid_size_m = _read_map(args.map_result)
        except (OSError, ValueError, TypeError, KeyError, AttributeError) as exc:
            result = _failure_result(
                batch.run_id,
                args.map_result,
                args.planner_result,
                "load_map",
                exc,
                input_ref=args.map_result,
                code="POSTPROCESS_MAP_INPUT_ERROR",
            )
        else:
            try:
                planner_entries = _read_planner(args.planner_result)
            except (OSError, ValueError, TypeError, KeyError, AttributeError) as exc:
                result = _failure_result(
                    batch.run_id,
                    args.map_result,
                    args.planner_result,
                    "load_planner",
                    exc,
                    input_ref=args.planner_result,
                    code="POSTPROCESS_PLANNER_INPUT_ERROR",
                )
            else:
                processed = []
                errors = []
                for entry in planner_entries:
                    processed_result = process_path(
                        grid,
                        entry["points"],
                        grid_size_m=grid_size_m,
                        start=start,
                        goal=goal,
                        parameters=parameters,
                        run_id=batch.run_id,
                        input_ref=args.planner_result,
                    )
                    processed.append(
                        {
                            "algorithm": entry["algorithm"],
                            "status": processed_result["status"],
                            "outputs": processed_result["outputs"],
                            "errors": processed_result["errors"],
                        }
                    )
                    errors.extend(processed_result["errors"])

                if len(processed) == 1:
                    result = make_result(
                        MODULE,
                        batch.run_id,
                        processed[0]["status"],
                        inputs={
                            "map_result": os.path.abspath(args.map_result),
                            "planner_result": os.path.abspath(args.planner_result),
                        },
                        outputs=processed[0]["outputs"],
                        parameters=parameters,
                        errors=errors,
                    )
                else:
                    result = make_result(
                        MODULE,
                        batch.run_id,
                        STATUS_FAILED if errors else STATUS_SUCCESS,
                        inputs={
                            "map_result": os.path.abspath(args.map_result),
                            "planner_result": os.path.abspath(args.planner_result),
                        },
                        outputs={"comparisons": processed},
                        parameters={**parameters, "algorithms": [item["algorithm"] for item in processed]},
                        errors=errors,
                    )

    log_path = batch.log_path(MODULE)
    with open(log_path, "a", encoding="utf-8") as log:
        log.write(
            f"run_id={batch.run_id} map={os.path.abspath(args.map_result)} "
            f"planner={os.path.abspath(args.planner_result)} "
            f"status={result['status']}\n"
        )
    result["logs"] = [log_path]
    write_result(output_path, result)
    batch.update_stage(
        MODULE,
        result["status"],
        result=output_path,
        error=result["errors"] if result["status"] == STATUS_FAILED else None,
    )
    batch.finish(result["status"])
    print(f"路径后处理 {result['status']}: {output_path}")
    return 0 if result["status"] == STATUS_SUCCESS else 2


if __name__ == "__main__":
    raise SystemExit(main())
