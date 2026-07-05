"""
基于快速随机树（RRT）的全局路径规划器（栅格版本）
在栅格地图上随机采样，逐步扩展树结构，直到找到连接目标的无碰撞路径。
"""

import math
import random
import numpy as np
from typing import List, Tuple
from ..planner_base import GlobalPlanner, Path

class RRTPlanner(GlobalPlanner):
    def __init__(self, grid: np.ndarray, max_iter: int = 2000, step_size: float = 5.0,
                 goal_sample_rate: float = 0.1):
        """
        :param grid: 栅格地图，0=空闲，1=障碍
        :param max_iter: 最大迭代次数
        :param step_size: 扩展步长（栅格单位）
        :param goal_sample_rate: 直接采样目标的概率（0~1）
        """
        super().__init__(grid)
        self.max_iter = max_iter
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.rows, self.cols = grid.shape

    def _sample_free(self):
        """随机采样一个自由栅格点"""
        while True:
            r = random.randint(0, self.rows - 1)
            c = random.randint(0, self.cols - 1)
            if self.grid[r, c] == 0:
                return (r, c)

    def _distance(self, p1, p2):
        return math.hypot(p1[0]-p2[0], p1[1]-p2[1])

    def _nearest_node(self, tree_nodes, point):
        """返回离 point 最近的坐标及其索引"""
        idx = min(range(len(tree_nodes)),
                key=lambda i: self._distance(tree_nodes[i][0], point))
        return tree_nodes[idx][0], idx   # 返回 (坐标, 索引)

    def _steer(self, from_node, to_point):
        """从 from_node 向 to_point 方向扩展一步，返回新坐标（可能是浮点，待取整）"""
        d = self._distance(from_node, to_point)
        if d <= self.step_size:
            return to_point
        ratio = self.step_size / d
        new_r = from_node[0] + ratio * (to_point[0] - from_node[0])
        new_c = from_node[1] + ratio * (to_point[1] - from_node[1])
        return (new_r, new_c)

    def _collision_free(self, p1, p2):
        """Bresenham 直线检测是否穿过障碍物"""
        r1, c1 = int(round(p1[0])), int(round(p1[1]))
        r2, c2 = int(round(p2[0])), int(round(p2[1]))
        dr = abs(r2 - r1)
        dc = abs(c2 - c1)
        sr = 1 if r1 < r2 else -1
        sc = 1 if c1 < c2 else -1
        err = dr - dc
        r, c = r1, c1
        while True:
            if not (0 <= r < self.rows and 0 <= c < self.cols):
                return False
            if self.grid[r, c] == 1:
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

    def plan(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Path:
        if not self.grid[start[0], start[1]] == 0 or not self.grid[goal[0], goal[1]] == 0:
            return []

        # 树存储为列表，每个元素是 (坐标, 父节点索引)
        tree_nodes = [(start, -1)]
        for _ in range(self.max_iter):
            # 按概率采样目标点
            if random.random() < self.goal_sample_rate:
                sample = goal
            else:
                sample = self._sample_free()

            nearest, nearest_idx = self._nearest_node(tree_nodes, sample)
            new_point = self._steer(nearest, sample)

            # 将浮点坐标转为栅格整数坐标
            new_r = int(round(new_point[0]))
            new_c = int(round(new_point[1]))
            new_r = max(0, min(self.rows-1, new_r))
            new_c = max(0, min(self.cols-1, new_c))

            # 检查整条线段是否无碰撞
            if self._collision_free(nearest, (new_r, new_c)):
                new_idx = len(tree_nodes)
                tree_nodes.append(((new_r, new_c), nearest_idx))

                # 检查是否到达目标附近
                if self._distance((new_r, new_c), goal) <= self.step_size:
                    if self._collision_free((new_r, new_c), goal):
                        # 将目标加入树
                        tree_nodes.append((goal, new_idx))
                        return self._extract_path(tree_nodes)

        return []   # 未找到路径

    def _extract_path(self, tree_nodes):
        path = []
        idx = len(tree_nodes) - 1
        while idx != -1:
            coord, parent = tree_nodes[idx]
            path.append(coord)
            idx = parent
        return path[::-1]