"""
栅格工具函数：碰撞检测等跨算法复用的纯函数。

提取自 prm.py / rrt.py / rrt_star.py 的三份重复 Bresenham 实现。
"""
from typing import Tuple
import numpy as np


def line_of_sight(grid: np.ndarray, rows: int, cols: int,
                  p1: Tuple[int, int], p2: Tuple[int, int]) -> bool:
    # A geometric edge must be safe in both traversal directions. Bresenham's
    # tie-breaking can otherwise visit different cells when endpoints reverse.
    return (_directional_line_of_sight(grid, rows, cols, p1, p2)
            and _directional_line_of_sight(grid, rows, cols, p2, p1))


def _directional_line_of_sight(grid: np.ndarray, rows: int, cols: int,
                               p1: Tuple[int, int], p2: Tuple[int, int]) -> bool:
    """
    Bresenham 线段栅格化，检查 p1→p2 直线是否不穿过障碍物。

    :param grid:  二值栅格，0=空闲 1=障碍
    :param rows:  栅格行数
    :param cols:  栅格列数
    :param p1:    起点 (row, col)，可以是整数或浮点（自动 round）
    :param p2:    终点 (row, col)，同上
    :return:      True = 视距畅通（全程无碰撞，未出界），False = 有碰撞或出界
    """
    r1, c1 = int(round(p1[0])), int(round(p1[1]))
    r2, c2 = int(round(p2[0])), int(round(p2[1]))
    dr = abs(r2 - r1)
    dc = abs(c2 - c1)
    sr = 1 if r1 < r2 else -1
    sc = 1 if c1 < c2 else -1
    err = dr - dc

    r, c = r1, c1
    while True:
        if not (0 <= r < rows and 0 <= c < cols):
            return False
        if grid[r, c] == 1:
            return False
        if r == r2 and c == c2:
            break
        e2 = 2 * err
        if e2 > -dc:
            err -= dc
            r += sr
        if e2 < dr:
            err += dr
            c += sc
    return True
