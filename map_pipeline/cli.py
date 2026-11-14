"""Command-line entry point for map processing."""

from __future__ import annotations

import argparse
import os
import sys
from typing import Sequence

import yaml

from core import (
    RunBatch,
    STATUS_FAILED,
    STATUS_SUCCESS,
    display_available,
    interactive_terminal,
    make_error,
    make_result,
    write_result,
)

try:
    from .interaction import (
        _interactive_points,
        _resolve_corners,
        _select_terminal_points,
    )
    from .process import prepare_grid_map, process
    from .src.coordinate_mapper import map_pixel_to_grid
    from .src.interactive_picker import PointSelectionCancelled
    from .src.map_loader import load_image
    from .src.preview import safe_imwrite
    from .src.rectify import rectify_image, transform_points
except ImportError:
    from interaction import (
        _interactive_points,
        _resolve_corners,
        _select_terminal_points,
    )
    from process import prepare_grid_map, process
    from src.coordinate_mapper import map_pixel_to_grid
    from src.interactive_picker import PointSelectionCancelled
    from src.map_loader import load_image
    from src.preview import safe_imwrite
    from src.rectify import rectify_image, transform_points

def _pair(value: str) -> tuple[int, int]:
    try:
        first, second = value.split(",", 1)
        return int(first), int(second)
    except (AttributeError, TypeError, ValueError) as exc:
        raise argparse.ArgumentTypeError("坐标格式应为 row,col 或 x,y") from exc

def _load_config(path: str) -> dict:
    with open(path, encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}
    return raw.get("map_pipeline", raw)

def _positive_float(value) -> float | None:
    if value is None:
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if number > 0 else None

def _positive_int(value) -> int | None:
    if value is None:
        return None
    try:
        number = int(value)
    except (TypeError, ValueError):
        return None
    return number if number > 0 else None

def _record_failed_batch(
    batch: RunBatch,
    output_path: str,
    *,
    image_ref: str,
    stage: str,
    error: Exception,
    parameters: dict | None = None,
    code: str = "MAP_INTERACTION_ERROR",
    suggestion: str = "检查地图文件、配置和起终点交互后重试",
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
                suggestion=suggestion,
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
        "--headless",
        action="store_true",
        help="不使用图形窗口；必须显式提供起终点或配合 --select-terminal",
    )
    parser.add_argument(
        "--select-terminal",
        action="store_true",
        help="在终端输入 row,col 起终点，不打开图形窗口",
    )
    parser.add_argument(
        "--rectify",
        action="store_true",
        help="先对原图做四点透视矫正和裁剪，再栅格化",
    )
    parser.add_argument(
        "--corners",
        help="四个场地角点 x,y;x,y;x,y;x,y（原图像素坐标）",
    )
    parser.add_argument(
        "--arena-width-m",
        type=float,
        help="场地真实宽度（米），用于计算栅格物理尺度",
    )
    parser.add_argument(
        "--arena-height-m",
        type=float,
        help="场地真实高度（米），用于确定矫正后长宽比",
    )
    parser.add_argument(
        "--rectify-output-width",
        type=int,
        help="矫正后图片宽度（像素）；省略时使用四边形平均宽度",
    )
    parser.add_argument(
        "--grid-size-m",
        type=float,
        help="每个栅格对应的真实边长（米），覆盖配置中的 grid_size",
    )
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
        if args.grid_size_m is not None:
            if args.grid_size_m <= 0:
                raise ValueError("--grid-size-m 必须为正数")
            config["grid_size"] = float(args.grid_size_m)
        image = load_image(args.image)
        working_image = image
        rectification: dict | None = None
        rectify_enabled = bool(
            args.rectify or args.corners or config.get("rectify", False)
        )
        if rectify_enabled:
            corners = _resolve_corners(args, image)
            arena_width_m = _positive_float(
                args.arena_width_m
                if args.arena_width_m is not None
                else config.get("arena_width_m")
            )
            arena_height_m = _positive_float(
                args.arena_height_m
                if args.arena_height_m is not None
                else config.get("arena_height_m")
            )
            output_width = _positive_int(
                args.rectify_output_width
                if args.rectify_output_width is not None
                else config.get("rectify_output_width")
            )
            working_image, rectification = rectify_image(
                image,
                corners,
                output_width=output_width,
                arena_width_m=arena_width_m,
                arena_height_m=arena_height_m,
            )
            rectified_path = os.path.splitext(output_path)[0] + "_rectified.png"
            safe_imwrite(rectified_path, working_image)
            rectification["image_path"] = os.path.abspath(rectified_path)
            print(
                "透视矫正: "
                f"{rectification['output_size']['width']}x"
                f"{rectification['output_size']['height']} -> "
                f"{rectification['image_path']}"
            )
            if arena_width_m is not None:
                desired_grid_size = _positive_float(
                    config.get("grid_size", 0.3)
                ) or 0.3
                target_cols = max(
                    1,
                    int(round(float(arena_width_m) / desired_grid_size)),
                )
                target_cols = min(
                    target_cols,
                    int(working_image.shape[1]),
                )
                config["default_width"] = target_cols
                config["grid_size"] = float(arena_width_m) / target_cols
                print(
                    f"物理尺度: {target_cols} 列, "
                    f"grid_size={config['grid_size']:.4f} m/cell"
                )

        if grid_pair_complete:
            start, goal, coordinate = args.start, args.goal, "grid"
        elif pixel_pair_complete:
            start, goal = args.start_px, args.goal_px
            if rectification is not None:
                transformed = transform_points(
                    [start, goal],
                    rectification["homography"],
                )
                start = tuple(int(round(value)) for value in transformed[0])
                goal = tuple(int(round(value)) for value in transformed[1])
            coordinate = "pixel"
        elif args.select_terminal:
            if not interactive_terminal():
                raise ValueError(
                    "--select-terminal 需要 stdin 和 stdout 都是交互式终端"
                )
            grid_map = prepare_grid_map(working_image, config)
            start, goal = _select_terminal_points(grid_map, preview_path)
            coordinate = "grid"
        elif not args.headless and interactive_terminal() and display_available():
            grid_map = prepare_grid_map(working_image, config)
            selected = _interactive_points(working_image, grid_map)
            start = map_pixel_to_grid(selected[0][0], selected[0][1], grid_map)
            goal = map_pixel_to_grid(selected[1][0], selected[1][1], grid_map)
            coordinate = "grid"
        else:
            raise ValueError(
                "无显示器或 --headless 模式必须同时提供 --start/--goal "
                "或 --start-px/--goal-px；"
                "也可使用 --headless --select-terminal 在终端输入起终点"
            )
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
            suggestion=(
                "无显示器时使用 --headless --select-terminal 在终端输入起终点，"
                "或显式提供 --start/--goal；否则检查图片和栅格参数"
            ),
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
        image_override=working_image,
        rectification=rectification,
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


__all__ = ["_record_failed_batch", "main"]
