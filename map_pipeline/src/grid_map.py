"""
栅格地图数据结构
"""

import numpy as np
from typing import Tuple

class GridMap:
    """封装栅格地图及基础查询"""

    def __init__(self, grid: np.ndarray, cell_width: int, cell_height: int,
                 img_shape: Tuple[int, int]):
        """
        :param grid: 二维 uint8 数组，0=空闲, 1=障碍
        :param cell_width: 每个栅格在原始图像中的像素宽度
        :param cell_height: 每个栅格在原始图像中的像素高度
        :param img_shape: 原始二值图的 (高, 宽)
        """
        self.grid = grid
        self.cell_width = cell_width
        self.cell_height = cell_height
        self.img_shape = img_shape
        self.rows, self.cols = grid.shape

    def is_free(self, row: int, col: int) -> bool:
        if 0 <= row < self.rows and 0 <= col < self.cols:
            return self.grid[row, col] == 0
        return False

    def find_nearest_free(self, row: int, col: int) -> Tuple[int, int]:
        """BFS 查找最近空闲格"""
        if self.is_free(row, col):
            return (row, col)
        visited = {(row, col)}
        queue = [(row, col)]
        while queue:
            r, c = queue.pop(0)
            for dr, dc in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                nr, nc = r + dr, c + dc
                if 0 <= nr < self.rows and 0 <= nc < self.cols and (nr, nc) not in visited:
                    if self.grid[nr, nc] == 0:
                        return (nr, nc)
                    visited.add((nr, nc))
                    queue.append((nr, nc))
        return (row, col)  # 理论上不会执行

    def save(self, filepath: str):
        """保存为 npz 文件"""
        np.savez(filepath,
                 grid=self.grid,
                 cell_width=self.cell_width,
                 cell_height=self.cell_height,
                 img_shape=np.array(self.img_shape))

    @staticmethod
    def load(filepath: str) -> 'GridMap':
        data = np.load(filepath)
        return GridMap(data['grid'],
                       int(data['cell_width']),
                       int(data['cell_height']),
                       tuple(data['img_shape']))