"""
基于 RRT* 的全局路径规划器（栅格版本）
在 RRT 基础上增加重选父节点和重布线，使路径渐进最优。
"""

import math
import random
import numpy as np
from typing import List, Tuple
from ..planner_base import GlobalPlanner, Path

class RRTStarPlanner(GlobalPlanner):
    def __init__(self, grid: np.ndarray, max_iter: int = 2000, step_size: float = 5.0,
                 goal_sample_rate: float = 0.1, search_radius: float = 10.0):
        super().__init__(grid)
        self.max_iter = max_iter
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.search_radius = search_radius
        self.rows, self.cols = grid.shape

    def _sample_free(self):
        while True:
            r = random.randint(0, self.rows - 1)
            c = random.randint(0, self.cols - 1)
            if self.grid[r, c] == 0:
                return (r, c)

    def _distance(self, p1, p2):
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

    def _nearest_node(self, tree_nodes, point):
        """返回离 point 最近的坐标及其索引（坐标, 索引）"""
        idx = min(range(len(tree_nodes)),
                  key=lambda i: self._distance(tree_nodes[i][0], point))
        return tree_nodes[idx][0], idx

    def _steer(self, from_node, to_point):
        d = self._distance(from_node, to_point)
        if d <= self.step_size:
            return to_point
        ratio = self.step_size / d
        new_r = from_node[0] + ratio * (to_point[0] - from_node[0])
        new_c = from_node[1] + ratio * (to_point[1] - from_node[1])
        return (new_r, new_c)

    def _collision_free(self, p1, p2):
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

    def _nearby_nodes(self, tree_nodes, point):
        indices = []
        for i, (coord, _, _) in enumerate(tree_nodes):
            if self._distance(coord, point) <= self.search_radius:
                indices.append(i)
        return indices

    def _choose_parent(self, tree_nodes, new_point, near_indices, nearest_idx):
        best_idx = nearest_idx
        best_cost = float('inf')
        for idx in near_indices:
            coord, cost, _ = tree_nodes[idx]
            if self._collision_free(coord, new_point):
                new_cost = cost + self._distance(coord, new_point)
                if new_cost < best_cost:
                    best_cost = new_cost
                    best_idx = idx
        return best_idx, best_cost

    def _rewire(self, tree_nodes, new_idx, near_indices):
        new_coord, new_cost, _ = tree_nodes[new_idx]
        for idx in near_indices:
            coord, old_cost, _ = tree_nodes[idx]
            d = self._distance(new_coord, coord)
            if new_cost + d < old_cost:
                if self._collision_free(new_coord, coord):
                    tree_nodes[idx] = (coord, new_cost + d, new_idx)

    def plan(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Path:
        if not self.grid[start[0], start[1]] == 0 or not self.grid[goal[0], goal[1]] == 0:
            return []

        # 树节点：(坐标, 累计代价, 父节点索引)
        tree_nodes = [(start, 0.0, -1)]
        best_path = None
        best_cost = float('inf')

        for _ in range(self.max_iter):
            if random.random() < self.goal_sample_rate:
                sample = goal
            else:
                sample = self._sample_free()

            nearest, nearest_idx = self._nearest_node(tree_nodes, sample)
            new_point = self._steer(nearest, sample)

            new_r = int(round(new_point[0]))
            new_c = int(round(new_point[1]))
            new_r = max(0, min(self.rows - 1, new_r))
            new_c = max(0, min(self.cols - 1, new_c))

            if not self._collision_free(nearest, (new_r, new_c)):
                continue

            near_indices = self._nearby_nodes(tree_nodes, (new_r, new_c))
            if not near_indices:
                near_indices = [nearest_idx]

            parent_idx, cost = self._choose_parent(tree_nodes, (new_r, new_c), near_indices, nearest_idx)
            if parent_idx is None:
                continue

            new_idx = len(tree_nodes)
            tree_nodes.append(((new_r, new_c), cost, parent_idx))

            self._rewire(tree_nodes, new_idx, near_indices)

            if self._distance((new_r, new_c), goal) <= self.step_size:
                if self._collision_free((new_r, new_c), goal):
                    goal_cost = cost + self._distance((new_r, new_c), goal)
                    if goal_cost < best_cost:
                        best_cost = goal_cost
                        tree_nodes.append((goal, goal_cost, new_idx))
                        best_path = self._extract_path(tree_nodes)
                        tree_nodes.pop()  # 删除临时目标节点，保持树结构

        return best_path if best_path else []

    def _extract_path(self, tree_nodes):
        path = []
        idx = len(tree_nodes) - 1
        while idx != -1:
            coord, _, parent = tree_nodes[idx]
            path.append(coord)
            idx = parent
        return path[::-1]