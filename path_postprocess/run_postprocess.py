#!/usr/bin/env python3
"""Run path post-processing from map and planner JSON results."""

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
    with open(path, encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}
    return raw.get("path_postprocess", raw)


def _read_map(path: str):
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
    result = require_success(path, module="path_planner")
    payload = result["outputs"].get("raw_path")
    if not isinstance(payload, dict):
        raise ValueError(
            "planner result must be a single successful result with outputs.raw_path"
        )
    points = payload.get("points", {}).get("values")
    if not isinstance(points, list):
        raise ValueError("planner result is missing outputs.raw_path.points.values")
    return points


def _failure_result(
    run_id: str,
    map_path: str,
    planner_path: str,
    stage: str,
    error: Exception,
) -> dict:
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
                "POSTPROCESS_INPUT_ERROR",
                module=MODULE,
                stage=stage,
                error_type=type(error).__name__,
                reason=str(error),
                input_ref=os.path.abspath(planner_path),
                suggestion="check that map and planner JSON results are successful",
            )
        ],
    )


def main(argv: Sequence[str] | None = None) -> int:
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
        grid, start, goal, grid_size_m = _read_map(args.map_result)
        raw_path = _read_planner(args.planner_result)
        result = process_path(
            grid,
            raw_path,
            grid_size_m=grid_size_m,
            start=start,
            goal=goal,
            parameters=parameters,
            run_id=batch.run_id,
            input_ref=args.planner_result,
        )
        result["inputs"]["map_result"] = os.path.abspath(args.map_result)
    except Exception as exc:
        result = _failure_result(
            batch.run_id,
            args.map_result,
            args.planner_result,
            "load_input",
            exc,
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
