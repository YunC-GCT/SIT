from __future__ import annotations

"""
基于 RRT* 的全局路径规划器（栅格版本）
继承 RRT，在其基础上增加重选父节点和重布线，使路径渐进最优。
"""

from typing import List, Tuple
from .rrt import RRTPlanner
from ..planner_base import Path
from ..grid_utils import line_of_sight


class RRTStarPlanner(RRTPlanner):
    """RRT* = RRT + choose_parent + rewire。继承 _sample_free / _distance / _nearest_node / _steer。"""

    def __init__(self, grid, max_iter: int = 2000, step_size: float = 5.0,
                 goal_sample_rate: float = 0.1, search_radius: float = 10.0,
                 seed: int | None = None):
        super().__init__(grid, max_iter=max_iter, step_size=step_size,
                         goal_sample_rate=goal_sample_rate, seed=seed)
        self.search_radius = search_radius

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
            if line_of_sight(self.grid, self.rows, self.cols, coord, new_point):
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
                if line_of_sight(self.grid, self.rows, self.cols, new_coord, coord):
                    tree_nodes[idx] = (coord, new_cost + d, new_idx)

    def plan(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Path:
        if not self.grid[start[0], start[1]] == 0 or not self.grid[goal[0], goal[1]] == 0:
            return []

        # 树节点：(坐标, 累计代价, 父节点索引)
        tree_nodes = [(start, 0.0, -1)]
        best_cost = float('inf')
        best_parent_idx = -1

        for _ in range(self.max_iter):
            if self.rng.random() < self.goal_sample_rate:
                sample = goal
            else:
                sample = self._sample_free()

            nearest, nearest_idx = self._nearest_node(tree_nodes, sample)
            new_point = self._steer(nearest, sample)

            new_r = int(round(new_point[0]))
            new_c = int(round(new_point[1]))
            new_r = max(0, min(self.rows - 1, new_r))
            new_c = max(0, min(self.cols - 1, new_c))

            if not line_of_sight(self.grid, self.rows, self.cols, nearest, (new_r, new_c)):
                continue

            near_indices = self._nearby_nodes(tree_nodes, (new_r, new_c))
            if not near_indices:
                near_indices = [nearest_idx]

            parent_idx, cost = self._choose_parent(tree_nodes, (new_r, new_c),
                                                    near_indices, nearest_idx)
            if parent_idx is None:
                continue

            new_idx = len(tree_nodes)
            tree_nodes.append(((new_r, new_c), cost, parent_idx))

            self._rewire(tree_nodes, new_idx, near_indices)

            # 检查是否可以直接连到 goal,记录最优连接点
            if self._distance((new_r, new_c), goal) <= self.step_size:
                if line_of_sight(self.grid, self.rows, self.cols, (new_r, new_c), goal):
                    candidate_cost = cost + self._distance((new_r, new_c), goal)
                    if candidate_cost < best_cost:
                        best_cost = candidate_cost
                        best_parent_idx = new_idx

        if best_parent_idx == -1:
            return []
        tree_nodes.append((goal, best_cost, best_parent_idx))
        best_path = self._extract_path(tree_nodes)
        tree_nodes.pop()
        return best_path

    def _extract_path(self, tree_nodes):
        """RRT* 树节点含 cost 字段，需覆盖 RRT 的简单版"""
        path = []
        idx = len(tree_nodes) - 1
        while idx != -1:
            coord, _, parent = tree_nodes[idx]
            path.append(coord)
            idx = parent
        return path[::-1]
