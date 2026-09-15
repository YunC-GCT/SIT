#!/usr/bin/env python3
"""将地图图片转换为统一 JSON 地图结果。

无显示器调用需要显式提供 ``--start row,col --goal row,col``，或使用
``--start-px x,y --goal-px x,y`` 提供原图像素坐标。预览图默认保存为
与结果 JSON 同名的 ``*_preview.png``。
"""

from __future__ import annotations

import argparse
import os
import sys
from numbers import Integral
from typing import Sequence

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

try:
    from .src.coordinate_mapper import map_pixel_to_grid, validate_grid_points
    from .src.interactive_picker import PointSelectionCancelled, pick_points
    from .src.map_loader import binarize, create_grid_map, load_image
    from .src.preview import save_grid_preview
except ImportError:
    from src.coordinate_mapper import map_pixel_to_grid, validate_grid_points
    from src.interactive_picker import PointSelectionCancelled, pick_points
    from src.map_loader import binarize, create_grid_map, load_image
    from src.preview import save_grid_preview

from core import (
    RunBatch,
    STATUS_FAILED,
    STATUS_SUCCESS,
    grid_map_payload,
    make_error,
    make_result,
    write_result,
)


def _pair(value: str) -> tuple[int, int]:
    try:
        first, second = value.split(",", 1)
        return int(first), int(second)
    except (AttributeError, TypeError, ValueError) as exc:
        raise argparse.ArgumentTypeError("坐标格式应为 row,col 或 x,y") from exc


def prepare_grid_map(image, config: dict):
    """按配置完成图像二值化和栅格化，供 GUI 与 JSON 入口复用。"""
    return create_grid_map(
        binarize(
            image,
            threshold=int(config.get("threshold", 0)),
            blur_ksize=int(config.get("blur_ksize", 5)),
            invert=bool(config.get("invert", False)),
        ),
        int(config.get("default_width", 500)),
        obstacle_ratio=float(config.get("obstacle_ratio", 0.5)),
        grid_size=float(config.get("grid_size", 0.30)),
    )[0]


def _validate_pixel_point(
    point: tuple[int, int],
    image_shape: tuple[int, ...],
    name: str,
) -> tuple[int, int]:
    x, y = point
    if not isinstance(x, Integral) or not isinstance(y, Integral):
        raise ValueError(f"{name} 像素坐标必须是整数")
    height, width = image_shape[:2]
    if not (0 <= x < width and 0 <= y < height):
        raise ValueError(
            f"{name} 像素坐标超出图片范围: ({x}, {y}), "
            f"shape=({height},{width})"
        )
    return int(x), int(y)


def process(
    image_path: str,
    *,
    start: tuple[int, int],
    goal: tuple[int, int],
    coordinate: str = "grid",
    config: dict | None = None,
    output_path: str | None = None,
    preview_path: str | None = None,
    run_id: str = "standalone",
) -> dict:
    """处理地图并返回、可选写入一个完整的地图模块 JSON 结果。"""
    cfg = dict(config or {})
    image_ref = os.path.abspath(image_path)
    try:
        if coordinate not in {"grid", "pixel"}:
            raise ValueError("coordinate 必须是 grid 或 pixel")

        image = load_image(image_path)
        if coordinate == "pixel":
            start = _validate_pixel_point(start, image.shape, "start")
            goal = _validate_pixel_point(goal, image.shape, "goal")

        grid_map = prepare_grid_map(image, cfg)
        if coordinate == "pixel":
            start = map_pixel_to_grid(start[0], start[1], grid_map)
            goal = map_pixel_to_grid(goal[0], goal[1], grid_map)
        start, goal = validate_grid_points(grid_map, start, goal)

        effective_preview_path = preview_path
        if effective_preview_path is None and output_path:
            effective_preview_path = os.path.splitext(output_path)[0] + "_preview.png"
        outputs = {
            "map": {
                **grid_map_payload(
                    grid_map.grid.astype(int).tolist(),
                    grid_size_m=float(grid_map.grid_size),
                    start=start,
                    goal=goal,
                ),
                "cell_width_px": grid_map.cell_width,
                "cell_height_px": grid_map.cell_height,
            }
        }
        if effective_preview_path:
            save_grid_preview(
                grid_map.grid,
                start,
                goal,
                effective_preview_path,
            )
            outputs["preview"] = {
                "path": os.path.abspath(effective_preview_path),
                "format": "png",
            }

        result = make_result(
            "map_pipeline",
            run_id,
            "success",
            inputs={
                "image": image_ref,
                "coordinate": coordinate,
            },
            outputs=outputs,
            parameters=cfg,
            metadata={
                "units": {"grid_size": "m/cell", "coordinates": "grid"},
            },
        )
    except Exception as exc:
        result = make_result(
            "map_pipeline",
            run_id,
            "failed",
            inputs={"image": image_ref, "coordinate": coordinate},
            parameters=cfg,
            errors=[
                make_error(
                    "MAP_PROCESSING_ERROR",
                    module="map_pipeline",
                    stage="process",
                    error_type=type(exc).__name__,
                    reason=str(exc),
                    input_ref=image_ref,
                    suggestion="检查图片、栅格参数以及起终点是否位于空闲栅格",
                )
            ],
        )

    if output_path:
        write_result(output_path, result)
    return result


def _interactive_points(image, grid_map):
    """GUI 选点适配器，统一使用严格的两点交互。"""
    return pick_points(image, grid_map=grid_map)


def _load_config(path: str) -> dict:
    with open(path, encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}
    return raw.get("map_pipeline", raw)


def _record_failed_batch(
    batch: RunBatch,
    output_path: str,
    *,
    image_ref: str,
    stage: str,
    error: Exception,
    parameters: dict | None = None,
    code: str = "MAP_INTERACTION_ERROR",
) -> dict:
    """Persist a structured failure and close the map stage."""
    result = make_result(
        "map_pipeline",
        batch.run_id,
        STATUS_FAILED,
        inputs={"image": image_ref},
        parameters=parameters or {},
        errors=[
            make_error(
                code,
                module="map_pipeline",
                stage=stage,
                error_type=type(error).__name__,
                reason=str(error),
                input_ref=image_ref,
                suggestion="检查地图文件、配置和起终点交互后重试",
            )
        ],
    )
    log_path = batch.log_path("map_pipeline")
    with open(log_path, "a", encoding="utf-8") as log:
        log.write(
            f"run_id={batch.run_id} input={image_ref} status={result['status']} "
            f"stage={stage}\n"
        )
    result["logs"] = [log_path]
    write_result(output_path, result)
    batch.update_stage(
        "map_pipeline",
        STATUS_FAILED,
        result=output_path,
        error=result["errors"],
    )
    batch.finish(STATUS_FAILED)
    return result


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="生成统一 JSON 地图结果")
    parser.add_argument("image")
    parser.add_argument("--start", type=_pair, help="栅格坐标 row,col")
    parser.add_argument("--goal", type=_pair, help="栅格坐标 row,col")
    parser.add_argument("--start-px", type=_pair, help="原图像素坐标 x,y")
    parser.add_argument("--goal-px", type=_pair, help="原图像素坐标 x,y")
    parser.add_argument(
        "--output",
        help="结果 JSON 路径；省略时写入 runs/<run-id>/map_pipeline/result.json",
    )
    parser.add_argument("--preview")
    parser.add_argument("--run-root", default="runs")
    parser.add_argument("--run-id")
    parser.add_argument(
        "--config",
        default=os.path.join(os.path.dirname(__file__), "config.yaml"),
    )
    args = parser.parse_args(argv)

    grid_pair_complete = args.start is not None and args.goal is not None
    pixel_pair_complete = args.start_px is not None and args.goal_px is not None
    if (
        (args.start is not None) != (args.goal is not None)
        or (args.start_px is not None) != (args.goal_px is not None)
        or (grid_pair_complete and pixel_pair_complete)
    ):
        parser.error("起点和终点必须成对提供，且只能选择 grid 或 pixel 一种坐标")

    batch = RunBatch(root=args.run_root, run_id=args.run_id).start()
    stage_dir = batch.stage_dir("map_pipeline")
    output_path = os.path.abspath(args.output) if args.output else os.path.join(stage_dir, "result.json")
    preview_path = (
        os.path.abspath(args.preview)
        if args.preview
        else os.path.splitext(output_path)[0] + "_preview.png"
    )

    try:
        config = _load_config(args.config)
        if grid_pair_complete:
            start, goal, coordinate = args.start, args.goal, "grid"
        elif pixel_pair_complete:
            start, goal, coordinate = args.start_px, args.goal_px, "pixel"
        else:
            if not sys.stdout.isatty():
                raise ValueError(
                    "无显示器模式必须同时提供 --start/--goal 或 --start-px/--goal-px"
                )
            image = load_image(args.image)
            grid_map = prepare_grid_map(image, config)
            start, goal = _interactive_points(image, grid_map)
            coordinate = "pixel"
    except PointSelectionCancelled as exc:
        result = _record_failed_batch(
            batch,
            output_path,
            image_ref=os.path.abspath(args.image),
            stage="select_points",
            error=exc,
            parameters=locals().get("config", {}),
            code="POINT_SELECTION_CANCELLED",
        )
        print(result["errors"][0]["reason"], file=sys.stderr)
        print(f"地图处理 failed: {output_path}", file=sys.stderr)
        return 2
    except Exception as exc:
        result = _record_failed_batch(
            batch,
            output_path,
            image_ref=os.path.abspath(args.image),
            stage="prepare_or_select",
            error=exc,
            parameters=locals().get("config", {}),
        )
        print(result["errors"][0]["reason"], file=sys.stderr)
        print(f"地图处理 failed: {output_path}", file=sys.stderr)
        return 2

    result = process(
        args.image,
        start=start,
        goal=goal,
        coordinate=coordinate,
        config=config,
        output_path=output_path,
        preview_path=preview_path,
        run_id=batch.run_id,
    )
    log_path = batch.log_path("map_pipeline")
    with open(log_path, "a", encoding="utf-8") as log:
        log.write(
            f"run_id={batch.run_id} input={os.path.abspath(args.image)} "
            f"status={result['status']}\n"
        )
    result["logs"] = [log_path]
    write_result(output_path, result)
    batch.update_stage(
        "map_pipeline",
        result["status"],
        result=output_path,
        error=result["errors"] if result["status"] == STATUS_FAILED else None,
    )
    batch.finish(result["status"])
    print(f"地图处理 {result['status']}: {output_path}")
    return 0 if result["status"] == STATUS_SUCCESS else 2


if __name__ == "__main__":
    raise SystemExit(main())
