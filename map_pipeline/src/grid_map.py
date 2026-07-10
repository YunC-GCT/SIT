"""
栅格地图数据结构
"""

import numpy as np
from collections import deque
from typing import Tuple

class GridMap:
    """封装栅格地图及基础查询"""

    def __init__(self, grid: np.ndarray, cell_width: int, cell_height: int,
                 img_shape: Tuple[int, int], grid_size: float = 0.3):
        """
        :param grid: 二维 uint8 数组，0=空闲, 1=障碍
        :param cell_width: 每个栅格在原始图像中的像素宽度
        :param cell_height: 每个栅格在原始图像中的像素高度
        :param img_shape: 原始二值图的 (高, 宽)
        :param grid_size: 每个栅格对应真实世界的物理尺寸（米），默认 0.3m
        """
        self.grid = grid
        self.cell_width = cell_width
        self.cell_height = cell_height
        self.img_shape = img_shape
        self.grid_size = grid_size
        self.rows, self.cols = grid.shape

    def is_free(self, row: int, col: int) -> bool:
        if 0 <= row < self.rows and 0 <= col < self.cols:
            return self.grid[row, col] == 0
        return False

    def find_nearest_free(self, row: int, col: int) -> Tuple[int, int]:
        """BFS 查找最近空闲格（O(n) 用 deque，避免 list.pop(0) 的 O(n²)）"""
        if self.is_free(row, col):
            return (row, col)
        visited = {(row, col)}
        queue = deque([(row, col)])
        while queue:
            r, c = queue.popleft()
            for dr, dc in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                nr, nc = r + dr, c + dc
                if 0 <= nr < self.rows and 0 <= nc < self.cols and (nr, nc) not in visited:
                    if self.grid[nr, nc] == 0:
                        return (nr, nc)
                    visited.add((nr, nc))
                    queue.append((nr, nc))
        # BFS 遍历完所有可达格子仍无空闲格：地图全是障碍物
        raise RuntimeError(
            f"find_nearest_free: 从 ({row}, {col}) BFS 遍历全图未找到空闲栅格，"
            f"地图可能全是障碍物")

    def save(self, filepath: str):
        """保存为 npz 文件"""
        np.savez(filepath,
                 grid=self.grid,
                 cell_width=self.cell_width,
                 cell_height=self.cell_height,
                 img_shape=np.array(self.img_shape),
                 grid_size=self.grid_size)

    @staticmethod
    def load(filepath: str) -> 'GridMap':
        data = np.load(filepath)
        return GridMap(data['grid'],
                       int(data['cell_width']),
                       int(data['cell_height']),
                       tuple(data['img_shape']),
                       grid_size=float(data.get('grid_size', 0.3)))