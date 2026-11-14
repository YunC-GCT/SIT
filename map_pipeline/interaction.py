"""Terminal and GUI point/corner selection helpers for map processing."""

from __future__ import annotations

import os

from core import display_available, interactive_terminal

try:
    from .src.interactive_picker import (
        PointSelectionCancelled,
        pick_points,
        pick_quad,
    )
    from .src.preview import save_grid_preview
    from .src.rectify import parse_corners, validate_corners
except ImportError:
    from src.interactive_picker import (
        PointSelectionCancelled,
        pick_points,
        pick_quad,
    )
    from src.preview import save_grid_preview
    from src.rectify import parse_corners, validate_corners

def _interactive_points(image, grid_map):
    """GUI 选点适配器，统一使用严格的两点交互。"""
    return pick_points(image, grid_map=grid_map)

def _parse_grid_point(value: str) -> tuple[int, int]:
    text = str(value).strip().replace("，", ",")
    parts = text.split(",")
    if len(parts) != 2:
        raise ValueError("grid point must use row,col")
    return int(parts[0].strip()), int(parts[1].strip())

def _prompt_grid_point(
    name: str,
    grid_map,
    *,
    input_fn=input,
    output_fn=print,
    forbidden: tuple[int, int] | None = None,
) -> tuple[int, int]:
    while True:
        try:
            raw = input_fn(f"{name} row,col: ")
        except EOFError as exc:
            raise PointSelectionCancelled(f"终端输入结束，已取消{name}选择") from exc
        except KeyboardInterrupt as exc:
            raise PointSelectionCancelled(f"已取消{name}选择") from exc
        if not raw.strip():
            raise PointSelectionCancelled(f"已取消{name}选择")
        try:
            row, col = _parse_grid_point(raw)
        except (TypeError, ValueError):
            output_fn("格式应为 row,col，例如 2,2")
            continue
        if not (0 <= row < grid_map.rows and 0 <= col < grid_map.cols):
            output_fn(
                f"超出范围：row 0..{grid_map.rows - 1}，"
                f"col 0..{grid_map.cols - 1}"
            )
            continue
        if not grid_map.is_free(row, col):
            output_fn("该格是障碍，请重新输入")
            continue
        if forbidden is not None and (row, col) == forbidden:
            output_fn("终点不能与起点相同")
            continue
        return row, col

def _select_terminal_points(
    grid_map,
    preview_path: str,
    *,
    input_fn=input,
    output_fn=print,
) -> tuple[tuple[int, int], tuple[int, int]]:
    """Print grid context and read start/goal coordinates from the terminal."""
    save_grid_preview(grid_map.grid, None, None, preview_path)
    output_fn(
        f"栅格地图: {grid_map.rows} 行 x {grid_map.cols} 列，"
        f"栅格边长 {grid_map.grid_size} m"
    )
    output_fn(f"无标记预览: {os.path.abspath(preview_path)}")
    output_fn("请输入栅格坐标 row,col（地图上方为北；直接回车取消）")
    start = _prompt_grid_point("起点", grid_map, input_fn=input_fn, output_fn=output_fn)
    goal = _prompt_grid_point(
        "终点",
        grid_map,
        input_fn=input_fn,
        output_fn=output_fn,
        forbidden=start,
    )
    return start, goal

def _parse_pixel_pair(value: str) -> tuple[int, int]:
    text = str(value).strip().replace("，", ",")
    parts = text.split(",")
    if len(parts) != 2:
        raise ValueError("pixel point must use x,y")
    return int(parts[0].strip()), int(parts[1].strip())

def _prompt_corners_terminal(
    image_shape,
    *,
    input_fn=input,
    output_fn=print,
) -> list[tuple[int, int]]:
    height, width = int(image_shape[0]), int(image_shape[1])
    output_fn(f"原图尺寸: {width} x {height}（x,y 像素坐标）")
    output_fn("按任意顺序输入四个场地角点，程序会自动排序")
    corners: list[tuple[int, int]] = []
    for label in ("第1个角", "第2个角", "第3个角", "第4个角"):
        while True:
            try:
                raw = input_fn(f"{label} x,y: ")
            except EOFError as exc:
                raise PointSelectionCancelled("终端输入结束，已取消四角选择") from exc
            except KeyboardInterrupt as exc:
                raise PointSelectionCancelled("已取消四角选择") from exc
            if not raw.strip():
                raise PointSelectionCancelled(f"已取消{label}选择")
            try:
                x, y = _parse_pixel_pair(raw)
            except (TypeError, ValueError):
                output_fn("格式应为 x,y，例如 30,20")
                continue
            if not (0 <= x < width and 0 <= y < height):
                output_fn(f"超出范围：x 0..{width - 1}，y 0..{height - 1}")
                continue
            corners.append((x, y))
            break
    return corners

def _resolve_corners(args, image) -> list[tuple[float, float]]:
    if args.corners:
        return validate_corners(parse_corners(args.corners), image.shape)
    if interactive_terminal() and display_available():
        try:
            return validate_corners(pick_quad(image), image.shape)
        except PointSelectionCancelled:
            raise
        except Exception as exc:
            print(f"图形四角选择不可用（{exc}），改用终端输入。")
    if interactive_terminal():
        return validate_corners(
            _prompt_corners_terminal(image.shape),
            image.shape,
        )
    raise ValueError(
        "透视矫正需要 --corners，或在交互式终端/图形界面中运行"
    )


__all__ = [
    "_interactive_points",
    "_select_terminal_points",
]
