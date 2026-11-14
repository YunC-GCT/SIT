#!/usr/bin/env python3
"""执行可重复的全流程及模拟 ROS2 验收。"""

from __future__ import annotations

import argparse
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from core.acceptance import run_simulated_acceptance


def _pair(value: str) -> tuple[int, int]:
    """解析验收命令行中的 row,col 整数坐标。"""
    try:
        row, col = value.split(",", 1)
        return int(row), int(col)
    except (AttributeError, TypeError, ValueError) as exc:
        raise argparse.ArgumentTypeError("coordinate must use row,col") from exc


def main(argv: list[str] | None = None) -> int:
    """解析验收选项，运行模拟验收并根据检查结果返回退出码。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", default=os.path.join(ROOT, "tests", "fixtures", "01_scattered.png"))
    parser.add_argument("--start", type=_pair, default=(2, 2))
    parser.add_argument("--goal", type=_pair, default=(37, 37))
    parser.add_argument("--mode", choices=("normal", "research"), default="normal")
    parser.add_argument("--algorithms", help="comma-separated names for research mode")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--run-root", default=os.path.join(ROOT, "runs"))
    parser.add_argument("--run-id")
    parser.add_argument("--map-config")
    parser.add_argument("--planner-config")
    parser.add_argument("--postprocess-config")
    args = parser.parse_args(argv)

    algorithms = (
        [name.strip() for name in args.algorithms.split(",") if name.strip()]
        if args.algorithms
        else None
    )
    result = run_simulated_acceptance(
        args.image,
        start=args.start,
        goal=args.goal,
        mode=args.mode,
        algorithms=algorithms,
        seed=args.seed,
        run_root=args.run_root,
        run_id=args.run_id,
        map_config=args.map_config,
        planner_config=args.planner_config,
        postprocess_config=args.postprocess_config,
    )
    print(
        f"Acceptance {result['status']}: "
        f"{result['outputs']['workflow']['manifest']}"
    )
    print(json.dumps(result["outputs"]["checks"], ensure_ascii=False, indent=2))
    return 0 if result["status"] == "success" else 2


if __name__ == "__main__":
    raise SystemExit(main())
