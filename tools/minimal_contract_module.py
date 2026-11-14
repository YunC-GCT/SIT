#!/usr/bin/env python3
"""最小统一 JSON 契约示例模块。

该脚本用于验证模块如何创建运行批次、写日志、输出成功/失败结果，并让
下游通过统一契约判断结果是否可继续使用。
"""

from __future__ import annotations

import argparse
import os
import sys
from typing import Sequence

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from core import (
    RunBatch,
    STATUS_FAILED,
    STATUS_SUCCESS,
    grid_map_payload,
    make_error,
    make_result,
    metric_payload,
    module_entry,
    path_payload,
    path_segment_payload,
    write_result,
)

MODULE = "contract_example"


def run_example(*, run_root: str, run_id: str | None = None, fail: bool = False) -> dict:
    """演示独立模块如何建立批次、写日志并输出统一成功或失败契约。"""
    batch = RunBatch(root=run_root, run_id=run_id).start()
    batch.register_module(MODULE)
    stage_dir = batch.stage_dir(MODULE)
    log_path = batch.log_path(MODULE)
    result_path = os.path.join(stage_dir, "result.json")

    with open(log_path, "a", encoding="utf-8") as log:
        log.write(f"run_id={batch.run_id} module={MODULE} start\n")
        if fail:
            error = make_error(
                "EXAMPLE_REQUESTED_FAILURE",
                module=MODULE,
                stage="example",
                error_type="requested_failure",
                reason="调用者请求生成失败示例",
                input_ref="--fail",
                suggestion="去掉 --fail 后重试，或保留该批次用于检查失败恢复流程",
            )
            result = make_result(
                MODULE,
                batch.run_id,
                STATUS_FAILED,
                inputs={"flag": "--fail"},
                errors=[error],
                logs=[log_path],
            )
            write_result(result_path, result)
            batch.update_stage(MODULE, STATUS_FAILED, result=result_path, error=error)
            batch.finish(STATUS_FAILED)
            log.write("status=failed\n")
            return result

        path = [(0, 0), (1, 0), (1, 1)]
        result = make_result(
            MODULE,
            batch.run_id,
            STATUS_SUCCESS,
            inputs={"source": "minimal_contract_module"},
            outputs={
                "map": grid_map_payload(
                    [[0, 0], [1, 0]],
                    grid_size_m=0.3,
                    start=path[0],
                    goal=path[-1],
                ),
                "raw_path": path_payload(path),
                "smoothed_path": path_payload(path),
                "segments": [
                    path_segment_payload(
                        0,
                        start=(0, 0),
                        end=(1, 0),
                        direction_deg=180.0,
                        turn_angle_deg=180.0,
                        grid_distance=1.0,
                        distance_m=0.3,
                        estimated_time_s=1.0,
                    ),
                    path_segment_payload(
                        1,
                        start=(1, 0),
                        end=(1, 1),
                        direction_deg=90.0,
                        turn_angle_deg=-90.0,
                        grid_distance=1.0,
                        distance_m=0.3,
                        estimated_time_s=1.0,
                    ),
                ],
                "metrics": metric_payload(
                    {
                        "found_path": True,
                        "raw_node_count": len(path),
                        "raw_path_length_m": 0.6,
                        "estimated_execution_time_s": 2.0,
                    },
                    {
                        "found_path": "boolean",
                        "raw_node_count": "count",
                        "raw_path_length_m": "m",
                        "estimated_execution_time_s": "s",
                    },
                ),
                "modules": [
                    module_entry(MODULE, STATUS_SUCCESS, result_path=result_path),
                ],
            },
            logs=[log_path],
        )
        write_result(result_path, result)
        batch.update_stage(MODULE, STATUS_SUCCESS, result=result_path)
        batch.finish(STATUS_SUCCESS)
        log.write("status=success\n")
        return result


def main(argv: Sequence[str] | None = None) -> int:
    """运行最小契约模块示例，将命令行参数交给示例 API 并返回退出码。"""
    parser = argparse.ArgumentParser(description="运行最小统一 JSON 契约示例模块")
    parser.add_argument("--run-root", default="runs")
    parser.add_argument("--run-id")
    parser.add_argument("--fail", action="store_true")
    args = parser.parse_args(argv)

    result = run_example(run_root=args.run_root, run_id=args.run_id, fail=args.fail)
    print(os.path.join(os.path.abspath(args.run_root), result["run_id"], MODULE, "result.json"))
    return 0 if result["status"] == STATUS_SUCCESS else 2


if __name__ == "__main__":
    raise SystemExit(main())
