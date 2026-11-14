from __future__ import annotations

"""
基于 RRT* 的全局路径规划器（栅格版本）
继承 RRT，在其基础上增加重选父节点和重布线，使路径渐进最优。
"""

import time
from typing import Tuple
from .rrt_adaptive import RRTAdaptivePlanner
from ..planner_base import Path
from ..grid_utils import line_of_sight


class RRTStarPlanner(RRTAdaptivePlanner):
    """RRT* = RRT + choose_parent + rewire。继承 _sample_free / _distance / _nearest_node / _steer。"""

    def __init__(self, grid, max_iter: int = 2000, step_size: float = 5.0,
                 goal_sample_rate: float = 0.1, search_radius: float = 10.0,
                 seed: int | None = None, adaptive_step: bool = False,
                 optimize_iterations: int | None = None, search_time_s: float = 0.0,
                 optimize_time_s: float = 0.0):
        super().__init__(grid, max_iter=max_iter, step_size=step_size,
                         goal_sample_rate=goal_sample_rate, seed=seed, adaptive_step=adaptive_step)
        self.search_radius = search_radius
        self.optimize_iterations = optimize_iterations
        self.search_time_s = search_time_s
        self.optimize_time_s = optimize_time_s
        self.diagnostics = {}

    def _nearby_nodes(self, tree_nodes, point):
        indices = []
        for i, (coord, _, _) in enumerate(tree_nodes):
            if self._distance(coord, point) <= self.search_radius:
                indices.append(i)
        return indices

    def _choose_parent(self, tree_nodes, new_point, near_indices, nearest_idx):
        best_idx = nearest_idx
        coord, cost, _ = tree_nodes[nearest_idx]
        best_cost = cost + self._distance(coord, new_point)
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
                    # All descendant costs must agree with their current parent chain.
                    children = {}
                    for child, (_, _, parent) in enumerate(tree_nodes):
                        children.setdefault(parent, []).append(child)
                    pending = [idx]
                    while pending:
                        parent = pending.pop()
                        for child in children.get(parent, []):
                            child_coord, _, _ = tree_nodes[child]
                            parent_coord, parent_cost, _ = tree_nodes[parent]
                            tree_nodes[child] = (child_coord, parent_cost + self._distance(parent_coord, child_coord), parent)
                            pending.append(child)

    def plan(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Path:
        if not self.grid[start[0], start[1]] == 0 or not self.grid[goal[0], goal[1]] == 0:
            return []
        if start == goal:
            return [start]

        # 树节点：(坐标, 累计代价, 父节点索引)
        tree_nodes = [(start, 0.0, -1)]
        goal_candidates = []
        if self._distance(start, goal) <= self.step_size and line_of_sight(self.grid, self.rows, self.cols, start, goal):
            goal_candidates.append(0)

        visited = {start}
        started = time.perf_counter()
        first_solution_iteration = 0 if goal_candidates else None
        first_solution_time = started if goal_candidates else None
        stop_reason = "max_iter"
        iterations = 0
        for iteration in range(self.max_iter):
            now = time.perf_counter()
            if first_solution_time is None and self.search_time_s and now-started >= self.search_time_s:
                stop_reason = "search_time"
                break
            if first_solution_time is not None:
                if self.optimize_iterations is not None and iteration-first_solution_iteration >= self.optimize_iterations:
                    stop_reason = "optimize_iterations"
                    break
                if self.optimize_time_s and now-first_solution_time >= self.optimize_time_s:
                    stop_reason = "optimize_time"
                    break
            iterations = iteration + 1
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

            if not line_of_sight(self.grid, self.rows, self.cols, nearest, (new_r, new_c)):
                continue

            near_indices = self._nearby_nodes(tree_nodes, (new_r, new_c))
            if not near_indices:
                near_indices = [nearest_idx]

            parent_idx, cost = self._choose_parent(tree_nodes, (new_r, new_c),
                                                    near_indices, nearest_idx)
            if parent_idx is None:
                continue

            visited.add((new_r, new_c))
            new_idx = len(tree_nodes)
            tree_nodes.append(((new_r, new_c), cost, parent_idx))

            self._rewire(tree_nodes, new_idx, near_indices)

            # 检查是否可以直接连到 goal,记录最优连接点
            if self._distance((new_r, new_c), goal) <= self.step_size:
                if line_of_sight(self.grid, self.rows, self.cols, (new_r, new_c), goal):
                    goal_candidates.append(new_idx)
                    if first_solution_time is None:
                        first_solution_iteration = iteration + 1
                        first_solution_time = time.perf_counter()

        self.diagnostics = dict(iterations=iterations, stop_reason=stop_reason,
            first_solution_iteration=first_solution_iteration,
            first_solution_s=None if first_solution_time is None else first_solution_time-started,
            total_s=time.perf_counter()-started, unique_nodes=len(visited))
        if not goal_candidates:
            return []
        best_parent_idx = min(goal_candidates, key=lambda idx: tree_nodes[idx][1] + self._distance(tree_nodes[idx][0], goal))
        best_cost = tree_nodes[best_parent_idx][1] + self._distance(tree_nodes[best_parent_idx][0], goal)
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
