#!/usr/bin/env python3
"""地图处理管线入口：选图、选点、栅格化并保存 JSON 结果。"""

from __future__ import annotations

import argparse
import os
import sys
from typing import Sequence

import cv2
import yaml

try:
    from .json_pipeline import (
        _record_failed_batch,
        _select_terminal_points,
        prepare_grid_map,
        process,
    )
    from .src.grid_resolution_tuner import tune_resolution
    from .src.interactive_picker import PointSelectionCancelled, pick_points
    from .src.preview import build_grid_preview, safe_imwrite
except ImportError:
    from json_pipeline import (
        _record_failed_batch,
        _select_terminal_points,
        prepare_grid_map,
        process,
    )
    from src.grid_resolution_tuner import tune_resolution
    from src.interactive_picker import PointSelectionCancelled, pick_points
    from src.preview import build_grid_preview, safe_imwrite

from core import (
    RunBatch,
    STATUS_FAILED,
    STATUS_SUCCESS,
    display_available,
    interactive_terminal,
)


def draw_grid_preview(
    grid_map,
    start_grid,
    goal_grid,
    save_path,
    cell_size=4,
    draw_grid_lines=True,
    show_window=True,
):
    """生成、保存并可选显示栅格预览图。"""
    image = build_grid_preview(
        grid_map,
        start_grid,
        goal_grid,
        cell_size=cell_size,
        draw_grid_lines=draw_grid_lines,
    )
    safe_imwrite(save_path, image)

    if show_window:
        window_name = "Grid Preview"
        max_width, max_height = 1000, 700
        image_height, image_width = image.shape[:2]
        scale = min(max_width / image_width, max_height / image_height, 1.0)
        display_width = max(1, int(image_width * scale))
        display_height = max(1, int(image_height * scale))
        display = cv2.resize(image, (display_width, display_height))
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, display_width, display_height)
        cv2.imshow(window_name, display)
        cv2.waitKey(1)
        cv2.destroyWindow(window_name)
    return image


def _load_config(path: str) -> dict:
    with open(path, encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}
    return raw.get("map_pipeline", raw)


def _find_image(input_dir: str, requested: str | None) -> str:
    if requested:
        return os.path.abspath(requested)
    extensions = (".jpg", ".jpeg", ".png", ".bmp")
    files = sorted(
        name
        for name in os.listdir(input_dir)
        if name.lower().endswith(extensions)
    )
    if not files:
        raise FileNotFoundError("data/input 目录下无图片，请放入图片后重试")
    if not sys.stdin.isatty():
        return os.path.join(input_dir, files[0])

    print("可用地图:")
    for index, name in enumerate(files, 1):
        print(f"  [{index}] {name}")
    choice = input("选择地图编号（直接回车使用第一张）: ").strip()
    if not choice:
        return os.path.join(input_dir, files[0])
    try:
        return os.path.join(input_dir, files[int(choice) - 1])
    except (ValueError, IndexError) as exc:
        raise ValueError("无效地图编号") from exc


def main(argv: Sequence[str] | None = None) -> int:
    base_dir = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(description="交互式地图处理")
    parser.add_argument("--image", help="地图图片路径；省略时使用 data/input 第一张图片")
    parser.add_argument("--config", default=os.path.join(base_dir, "config.yaml"))
    parser.add_argument(
        "--run-id",
        help="运行批次编号；省略时自动生成",
    )
    parser.add_argument(
        "--output",
        help="JSON 结果路径；省略时写入 runs/<run-id>/map_pipeline/result.json",
    )
    parser.add_argument("--preview", help="预览图路径；省略时写入对应运行批次")
    parser.add_argument("--run-root", default="runs")
    parser.add_argument(
        "--adjust-grid",
        action="store_true",
        help="主动进入终端栅格宽度设置；默认直接使用配置值",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="不打开图形窗口；需配合 --select-terminal",
    )
    parser.add_argument(
        "--select-terminal",
        action="store_true",
        help="在终端输入 row,col 起终点",
    )
    args = parser.parse_args(argv)
    image_path = args.image or ""

    batch = RunBatch(root=args.run_root, run_id=args.run_id).start()
    stage_dir = batch.stage_dir("map_pipeline")
    result_path = os.path.abspath(args.output) if args.output else os.path.join(stage_dir, "result.json")
    preview_path = (
        os.path.abspath(args.preview)
        if args.preview
        else os.path.splitext(result_path)[0] + "_preview.png"
    )
    try:
        config = _load_config(args.config)
        input_dir = os.path.join(base_dir, "data", "input")
        image_path = _find_image(input_dir, args.image)
        image = _load_image(image_path)
        image_height, image_width = image.shape[:2]
        target_width = int(config.get("default_width", 500))

        if args.adjust_grid:
            target_width = tune_resolution(
                target_width,
                image_width,
                image_height,
                lambda width: (
                    max(1, int(image_height / (image_width / width))),
                    width,
                ),
            )

        effective_config = {**config, "default_width": target_width}
        grid_map = prepare_grid_map(image, effective_config)

        print(f"使用图片: {image_path}")
        print(f"图片尺寸: {image_width} x {image_height}")
        print(f"栅格尺寸: {grid_map.rows} 行 x {grid_map.cols} 列")
        terminal_ok = interactive_terminal()
        gui_ok = terminal_ok and display_available()
        if args.select_terminal or (not args.headless and not gui_ok and terminal_ok):
            start, goal = _select_terminal_points(grid_map, preview_path)
            coordinate = "grid"
        elif not args.headless and gui_ok:
            start, goal = pick_points(image, grid_map=grid_map)
            coordinate = "pixel"
        else:
            raise ValueError(
                "无显示器模式需要起终点：请使用 --select-terminal 在终端输入"
            )
    except PointSelectionCancelled as exc:
        result = _record_failed_batch(
            batch,
            result_path,
            image_ref=(
                os.path.abspath(image_path)
                if "image_path" in locals()
                else "data/input"
            ),
            stage="select_points",
            error=exc,
            parameters=locals().get("effective_config", {}),
            code="POINT_SELECTION_CANCELLED",
        )
        print(f"地图处理 failed: {result_path}")
        return 2
    except Exception as exc:
        image_ref = os.path.abspath(args.image) if args.image else "data/input"
        result = _record_failed_batch(
            batch,
            result_path,
            image_ref=image_ref,
            stage="prepare_or_select",
            error=exc,
            parameters=locals().get("effective_config", {}),
        )
        print(f"地图处理 failed: {result_path}")
        return 2

    result = process(
        image_path,
        start=start,
        goal=goal,
        coordinate=coordinate,
        config=effective_config,
        output_path=result_path,
        preview_path=preview_path,
        run_id=batch.run_id,
    )
    batch.update_stage(
        "map_pipeline",
        result["status"],
        result=result_path,
        error=result["errors"] if result["status"] == STATUS_FAILED else None,
    )
    batch.finish(result["status"])
    if result["status"] != STATUS_SUCCESS:
        print(f"地图处理 failed: {result_path}")
        print(result["errors"][0]["reason"])
        return 2

    print(f"地图处理 success: {result_path}")
    print(f"  预览图: {preview_path}")
    return 0


def _load_image(image_path: str):
    try:
        from .src.map_loader import load_image
    except ImportError:
        from src.map_loader import load_image
    return load_image(image_path)


if __name__ == "__main__":
    raise SystemExit(main())
