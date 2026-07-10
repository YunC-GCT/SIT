"""
图片加载、二值化、栅格化
"""

import os
import cv2
import numpy as np
from .grid_map import GridMap
from typing import Tuple

def load_image(image_path: str) -> np.ndarray:
    """读取图片，兼容中文路径，返回 BGR 图像"""
    if not os.path.exists(image_path):
        raise FileNotFoundError(f"图片不存在: {image_path}")
    img = cv2.imdecode(np.fromfile(image_path, dtype=np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError(f"无法读取图片: {image_path}")
    return img

def binarize(img: np.ndarray, threshold: int = 0, blur_ksize: int = 5,
             invert: bool = False) -> np.ndarray:
    """
    灰度 + 高斯模糊 + 阈值处理 → 二值图 (0=障碍, 255=空闲)
    threshold=0 时使用 OTSU 自动阈值
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (blur_ksize, blur_ksize), 0)
    if threshold == 0:
        # OTSU 自动阈值
        _, binary = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    else:
        _, binary = cv2.threshold(blurred, threshold, 255, cv2.THRESH_BINARY)
    if invert:
        binary = cv2.bitwise_not(binary)
    return binary

def create_grid_map(binary: np.ndarray, target_width: int,
                    obstacle_ratio: float = 0.5,
                    grid_size: float = 0.3) -> Tuple[GridMap, float]:
    """
    根据目标栅格宽度（列数）自动计算行数（保持宽高比），生成栅格地图
    :returns: (GridMap, scale)  scale = 原图宽度 / 栅格列数
    """
    img_h, img_w = binary.shape
    # 计算列数与行数
    grid_cols = target_width
    cell_w = img_w / grid_cols
    cell_h = cell_w  # 保持正方形栅格
    grid_rows = max(1, int(img_h / cell_h))
    # 实际使用的图像区域（右下角可能裁掉余数）
    used_h = int(grid_rows * cell_h)
    used_w = int(grid_cols * cell_w)
    binary_cropped = binary[:used_h, :used_w]
    grid_map = np.zeros((grid_rows, grid_cols), dtype=np.uint8)
    for i in range(grid_rows):
        for j in range(grid_cols):
            cell = binary_cropped[int(i * cell_h):int((i + 1) * cell_h),
                                  int(j * cell_w):int((j + 1) * cell_w)]
            # 黑色(0)为障碍，计算黑色占比
            black_ratio = np.sum(cell == 0) / cell.size
            if black_ratio > obstacle_ratio:
                grid_map[i, j] = 1
    return GridMap(grid_map, int(cell_w), int(cell_h), (used_h, used_w),
                   grid_size=grid_size), cell_w