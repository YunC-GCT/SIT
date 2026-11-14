"""Map image binarization, gridding, and result construction."""

from __future__ import annotations

import os
from numbers import Integral

from core import grid_map_payload, make_error, make_result, write_result

try:
    from .src.coordinate_mapper import map_pixel_to_grid, validate_grid_points
    from .src.map_loader import binarize, clean_binary, create_grid_map, load_image
    from .src.preview import save_grid_preview
except ImportError:
    from src.coordinate_mapper import map_pixel_to_grid, validate_grid_points
    from src.map_loader import binarize, clean_binary, create_grid_map, load_image
    from src.preview import save_grid_preview

def prepare_grid_map(image, config: dict):
    """按配置完成图像二值化和栅格化，供 GUI 与 JSON 入口复用。"""
    image_width = int(image.shape[1])
    configured_width = int(config.get("default_width", 500))
    target_width = min(configured_width, image_width)
    binary = binarize(
        image,
        threshold=int(config.get("threshold", 0)),
        blur_ksize=int(config.get("blur_ksize", 5)),
        invert=bool(config.get("invert", False)),
    )
    binary = clean_binary(
        binary,
        min_area_px=int(config.get("min_obstacle_area_px", 0)),
        morphology_kernel=int(config.get("morphology_kernel", 0)),
    )
    return create_grid_map(
        binary,
        target_width,
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
    image_override=None,
    rectification: dict | None = None,
) -> dict:
    """处理地图并返回、可选写入一个完整的地图模块 JSON 结果。"""
    cfg = dict(config or {})
    image_ref = os.path.abspath(image_path)
    try:
        if coordinate not in {"grid", "pixel"}:
            raise ValueError("coordinate 必须是 grid 或 pixel")

        image = image_override if image_override is not None else load_image(image_path)
        cfg["default_width"] = min(
            int(cfg.get("default_width", 500)),
            int(image.shape[1]),
        )
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
        if rectification is not None:
            outputs["rectification"] = rectification
            rectified_image_path = rectification.get("image_path")
            if rectified_image_path:
                outputs["rectified_image"] = {
                    "path": os.path.abspath(rectified_image_path),
                    "format": "png",
                }

        result = make_result(
            "map_pipeline",
            run_id,
            "success",
            inputs={
                "image": image_ref,
                "coordinate": coordinate,
                "rectified": bool(rectification),
            },
            outputs=outputs,
            parameters=cfg,
            metadata={
                "units": {"grid_size": "m/cell", "coordinates": "grid"},
                "rectified": bool(rectification),
            },
        )
    except Exception as exc:
        result = make_result(
            "map_pipeline",
            run_id,
            "failed",
            inputs={
                "image": image_ref,
                "coordinate": coordinate,
                "rectified": bool(rectification),
            },
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


__all__ = ["prepare_grid_map", "process"]
