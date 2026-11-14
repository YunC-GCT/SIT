#!/usr/bin/env python3
"""从地图 JSON 结果运行一个或多个规划算法。"""

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
    make_error,
    make_result,
    require_success,
    write_result,
)
from .src.planner_factory import list_algorithms  # noqa: E402
from .src.planner_pipeline import run_planning  # noqa: E402


def _parse_row_col(value: str) -> tuple[int, int]:
    """解析 CLI 的 row,col 文本为整数坐标。"""
    try:
        first, second = value.split(",", 1)
        return int(first), int(second)
    except (AttributeError, TypeError, ValueError) as exc:
        raise argparse.ArgumentTypeError("坐标格式应为 row,col") from exc


def _load_config(path: str) -> dict:
    """读取规划器 YAML 配置并返回字典。"""
    with open(path, encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}
    return raw.get("path_planner", raw)


def _read_map(path: str):
    """读取成功的地图模块结果，提取栅格、起终点及物理尺度。"""
    result = require_success(path, module="map_pipeline")
    map_payload = result["outputs"].get("map")
    if not isinstance(map_payload, dict):
        raise ValueError("地图结果缺少 outputs.map")
    grid_payload = map_payload.get("grid", {})
    grid = np.asarray(grid_payload.get("values"))
    start_payload = map_payload.get("start")
    goal_payload = map_payload.get("goal")
    if not start_payload or not goal_payload:
        raise ValueError("地图结果必须包含 start 和 goal")
    start = (int(start_payload["row"]), int(start_payload["col"]))
    goal = (int(goal_payload["row"]), int(goal_payload["col"]))
    return grid, start, goal, float(map_payload["grid_size_m"])


def _parameters_from_cli(
    selected: list[str],
    config: dict,
    raw_json: str | None,
) -> dict[str, dict]:
    """汇总 CLI 提供的算法参数，形成覆盖默认配置的参数字典。"""
    values = {
        name: dict(config.get("algorithm_params", {}).get(name, {}) or {})
        for name in selected
    }
    if not raw_json:
        return values
    try:
        override = json.loads(raw_json)
    except json.JSONDecodeError as exc:
        raise ValueError(f"--params 必须是 JSON 对象: {exc}") from exc
    if not isinstance(override, dict):
        raise ValueError("--params 必须是 JSON 对象")
    if any(isinstance(value, dict) for value in override.values()):
        for name, params in override.items():
            if name not in values:
                raise ValueError(f"--params 包含未选择的算法: {name}")
            if not isinstance(params, dict):
                raise ValueError(f"--params.{name} 必须是 JSON 对象")
            values[name].update(params)
    else:
        for name in selected:
            values[name].update(override)
    return values


def _failure_result(
    run_id: str,
    map_path: str,
    stage: str,
    exc: Exception,
) -> dict:
    """将规划入口异常封装为包含输入及恢复建议的失败结果。"""
    return make_result(
        "path_planner",
        run_id,
        STATUS_FAILED,
        inputs={"map_result": os.path.abspath(map_path)},
        errors=[
            make_error(
                "PLANNER_INPUT_ERROR",
                module="path_planner",
                stage=stage,
                error_type=type(exc).__name__,
                reason=str(exc),
                input_ref=os.path.abspath(map_path),
                suggestion="检查地图 JSON 是否为 map_pipeline 的 success 结果",
            )
        ],
    )


def main(argv: Sequence[str] | None = None) -> int:
    """运行规划 CLI，加载地图与配置，执行单算法或对照实验并保存统一结果。"""
    parser = argparse.ArgumentParser(description="生成统一 JSON 规划结果")
    parser.add_argument("--map-result", required=True, help="map_pipeline 的 success JSON")
    parser.add_argument("--algorithm", default=None, help="单个算法名称")
    parser.add_argument("--algorithms", help="逗号分隔的多个算法名称")
    parser.add_argument("--research", action="store_true", help="运行比较模式")
    parser.add_argument("--start", type=_parse_row_col, help="覆盖地图结果中的起点 row,col")
    parser.add_argument("--goal", type=_parse_row_col, help="覆盖地图结果中的终点 row,col")
    parser.add_argument("--seed", type=int)
    parser.add_argument("--params", help="算法参数 JSON 对象")
    parser.add_argument("--output", help="结果 JSON 路径")
    parser.add_argument("--run-root", default="runs")
    parser.add_argument("--run-id")
    parser.add_argument(
        "--config",
        default=os.path.join(os.path.dirname(__file__), "config.yaml"),
    )
    args = parser.parse_args(argv)

    batch = RunBatch(root=args.run_root, run_id=args.run_id).start()
    batch.register_module("path_planner")
    output_path = os.path.abspath(args.output) if args.output else os.path.join(
        batch.stage_dir("path_planner"), "result.json"
    )

    try:
        config = _load_config(args.config)
        grid, map_start, map_goal, grid_size_m = _read_map(args.map_result)
        if (args.start is None) != (args.goal is None):
            raise ValueError("--start 和 --goal 必须成对提供")
        start, goal = (
            (args.start, args.goal)
            if args.start is not None
            else (map_start, map_goal)
        )
        if args.algorithms and args.algorithm:
            raise ValueError("--algorithm 和 --algorithms 不能同时提供")
        if args.algorithms:
            selected = [name.strip() for name in args.algorithms.split(",") if name.strip()]
        elif args.algorithm:
            selected = [args.algorithm]
        elif args.research:
            selected = list(config.get("available_algorithms", list_algorithms()))
        else:
            selected = [config.get("default_algorithm", "astar")]
        parameters = _parameters_from_cli(selected, config, args.params)
        mode = "research" if args.research or len(selected) > 1 else "single"
        result = run_planning(
            grid,
            start,
            goal,
            selected,
            parameters=parameters,
            seed=args.seed,
            grid_size_m=grid_size_m,
            run_id=batch.run_id,
            input_ref=args.map_result,
            mode=mode,
        )
    except Exception as exc:
        result = _failure_result(batch.run_id, args.map_result, "load_input", exc)

    log_path = batch.log_path("path_planner")
    with open(log_path, "a", encoding="utf-8") as log:
        log.write(
            f"run_id={batch.run_id} input={os.path.abspath(args.map_result)} "
            f"status={result['status']}\n"
        )
    result["logs"] = [log_path]
    write_result(output_path, result)
    batch.update_stage(
        "path_planner",
        result["status"],
        result=output_path,
        error=result["errors"] if result["status"] == STATUS_FAILED else None,
    )
    batch.finish(result["status"])
    print(f"路径规划 {result['status']}: {output_path}")
    return 0 if result["status"] == "success" else 2


if __name__ == "__main__":
    raise SystemExit(main())
