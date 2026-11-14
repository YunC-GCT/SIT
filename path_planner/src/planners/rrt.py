from __future__ import annotations

"""
基于快速随机树（RRT）的全局路径规划器（栅格版本）
在栅格地图上随机采样，逐步扩展树结构，直到找到连接目标的无碰撞路径。
"""

import math
import numpy as np
from typing import Tuple
from ..planner_base import GlobalPlanner, Path
from ..grid_utils import line_of_sight

class RRTPlanner(GlobalPlanner):
    def __init__(self, grid: np.ndarray, max_iter: int = 2000, step_size: float = 5.0,
                 goal_sample_rate: float = 0.1, seed: int | None = None):
        """
        :param grid: 栅格地图，0=空闲，1=障碍
        :param max_iter: 最大迭代次数
        :param step_size: 扩展步长（栅格单位）
        :param goal_sample_rate: 直接采样目标的概率（0~1）
        """
        super().__init__(grid, seed=seed)
        self.max_iter = max_iter
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.rows, self.cols = grid.shape

    def _sample_free(self):
        """随机采样一个自由栅格点，加入迭代上限防止全障碍物时死循环"""
        max_attempts = self.rows * self.cols * 10
        for _ in range(max_attempts):
            r = self.rng.randint(0, self.rows - 1)
            c = self.rng.randint(0, self.cols - 1)
            if self.grid[r, c] == 0:
                return (r, c)
        # 兜底：线性扫描找第一个自由格子
        for r in range(self.rows):
            for c in range(self.cols):
                if self.grid[r, c] == 0:
                    return (r, c)
        raise RuntimeError("地图中没有空闲栅格，无法采样")

    def _distance(self, p1, p2):
        """计算两个行列坐标之间的欧氏距离，单位为栅格。"""
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


    def _extension(self, nearest, sample):
        """按固定步长扩展并取整；连线穿障或越界时返回 None。"""
        point = self._steer(nearest, sample)
        point = (int(round(point[0])), int(round(point[1])))
        return point if line_of_sight(self.grid, self.rows, self.cols, nearest, point) else None

    def plan(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Path:
        """运行固定步长、单树基础 RRT，按目标采样概率探索并检查新边碰撞。

        找到可连接目标的节点后回溯路径；迭代预算耗尽时返回空列表。"""
        if not self.grid[start[0], start[1]] == 0 or not self.grid[goal[0], goal[1]] == 0:
            return []
        if start == goal:
            return [start]

        # 树存储为列表，每个元素是 (坐标, 父节点索引)
        tree_nodes = [(start, -1)]
        visited = {start}
        for _ in range(self.max_iter):
            # 按概率采样目标点
            if self.rng.random() < self.goal_sample_rate:
                sample = goal
            else:
                sample = self._sample_free()

            nearest, nearest_idx = self._nearest_node(tree_nodes, sample)
            point = self._extension(nearest, sample)
            if point is None:
                continue
            new_r, new_c = point

            if (new_r, new_c) in visited:
                continue

            # 检查整条线段是否无碰撞
            if line_of_sight(self.grid, self.rows, self.cols, nearest, (new_r, new_c)):
                visited.add((new_r, new_c))
                new_idx = len(tree_nodes)
                tree_nodes.append(((new_r, new_c), nearest_idx))

                # 检查是否到达目标附近
                if self._distance((new_r, new_c), goal) <= self.step_size:
                    if line_of_sight(self.grid, self.rows, self.cols, (new_r, new_c), goal):
                        # 将目标加入树
                        tree_nodes.append((goal, new_idx))
                        return self._extract_path(tree_nodes)

        return []   # 未找到路径

    def _extract_path(self, tree_nodes):
        """沿父节点索引逆向回溯，再反转为起点到终点的路径。"""
        path = []
        idx = len(tree_nodes) - 1
        while idx != -1:
            coord, parent = tree_nodes[idx]
            path.append(coord)
            idx = parent
        return path[::-1]
