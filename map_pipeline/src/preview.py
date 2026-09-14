"""栅格地图预览图的生成和保存。"""

from __future__ import annotations

import os

import cv2
import numpy as np


def safe_imwrite(filepath: str, image: np.ndarray) -> str:
    """保存图片并兼容 Windows 中文路径。"""
    directory = os.path.dirname(os.path.abspath(filepath))
    os.makedirs(directory, exist_ok=True)
    extension = os.path.splitext(filepath)[1] or ".png"
    encoded_ok, encoded = cv2.imencode(extension, image)
    if not encoded_ok:
        raise ValueError(f"无法编码预览图片: {filepath}")
    with open(filepath, "wb") as handle:
        handle.write(encoded.tobytes())
    return filepath


def build_grid_preview(
    grid_map: np.ndarray,
    start_grid: tuple[int, int],
    goal_grid: tuple[int, int],
    *,
    cell_size: int = 4,
    draw_grid_lines: bool = True,
) -> np.ndarray:
    """生成带障碍、栅格线和起终点标记的预览图。"""
    if not isinstance(grid_map, np.ndarray) or grid_map.ndim != 2:
        raise ValueError("grid_map 必须是二维 numpy 数组")
    if cell_size < 1:
        raise ValueError("cell_size 必须为正整数")

    rows, cols = grid_map.shape
    image = np.ones((rows * cell_size, cols * cell_size, 3), dtype=np.uint8) * 255

    for row in range(rows):
        for col in range(cols):
            if grid_map[row, col] == 1:
                top = row * cell_size
                left = col * cell_size
                cv2.rectangle(
                    image,
                    (left, top),
                    ((col + 1) * cell_size - 1, (row + 1) * cell_size - 1),
                    (0, 0, 0),
                    -1,
                )

    if draw_grid_lines:
        for row in range(1, rows):
            y = row * cell_size
            cv2.line(image, (0, y), (cols * cell_size, y), (200, 200, 200), 1)
        for col in range(1, cols):
            x = col * cell_size
            cv2.line(image, (x, 0), (x, rows * cell_size), (200, 200, 200), 1)

    for point, color in (
        (start_grid, (0, 200, 0)),
        (goal_grid, (0, 0, 220)),
    ):
        row, col = point
        center = (
            int((col + 0.5) * cell_size),
            int((row + 0.5) * cell_size),
        )
        cv2.circle(image, center, max(2, cell_size // 2), color, -1)

    return image


def save_grid_preview(
    grid_map: np.ndarray,
    start_grid: tuple[int, int],
    goal_grid: tuple[int, int],
    save_path: str,
    *,
    cell_size: int = 4,
    draw_grid_lines: bool = True,
) -> str:
    """生成并保存栅格预览图，返回绝对路径。"""
    image = build_grid_preview(
        grid_map,
        start_grid,
        goal_grid,
        cell_size=cell_size,
        draw_grid_lines=draw_grid_lines,
    )
    return safe_imwrite(save_path, image)
