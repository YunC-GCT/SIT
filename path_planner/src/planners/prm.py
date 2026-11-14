from __future__ import annotations

"""
基于概率路图（PRM）的全局路径规划器（栅格版本）
直接在栅格地图上采样自由点、建立路图，并用 Dijkstra 搜索路径。
"""

import math
import heapq
import numpy as np
from typing import Tuple
from ..planner_base import GlobalPlanner, Path
from ..grid_utils import line_of_sight

class PRMPlanner(GlobalPlanner):
    def __init__(self, grid: np.ndarray, num_samples: int = 500, k_neighbors: int = 10,
                 max_edge_len: float = 30.0, seed: int | None = None,
                 retry_num_samples: int = 0):
        """
        :param grid: 栅格地图，0=空闲，1=障碍
        :param num_samples: 随机采样点数量（不含起终点）
        :param k_neighbors: 每个点主动选择的可通视近邻数（无向反向边可使度数更高）
        :param max_edge_len: 单条边的最大长度（栅格单元数），超过则不连接
        """
        super().__init__(grid, seed=seed)
        self.retry_num_samples = retry_num_samples
        self.diagnostics = {}
        self.num_samples = num_samples
        self.k_neighbors = k_neighbors
        self.max_edge_len = max_edge_len
        self.rows, self.cols = grid.shape

    def _sample_free_points(self, start, goal):
        """在自由栅格中随机采样，并添加起终点"""
        endpoints = list(dict.fromkeys(point for point in (start, goal) if self.grid[point] == 0))
        free = [tuple(map(int, point)) for point in np.argwhere(self.grid == 0)
                if tuple(point) not in endpoints]
        return endpoints + self.rng.sample(free, min(self.num_samples, len(free)))

    def _distance(self, p1, p2):
        """两点之间的欧氏距离（栅格坐标）"""
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

    def _build_roadmap(self, points):
        """
        为所有采样点建立邻接表形式的 roadmap。
        :return: road_map (每个点的邻居列表索引), points (最终使用的点列表)
        """
        # 按最大连边半径划分空间桶，避免对全部点两两排序。
        buckets = {}
        width = self.max_edge_len
        for i, (r, c) in enumerate(points):
            buckets.setdefault((int(r // width), int(c // width)), []).append(i)
        road_map = [set() for _ in points]
        visibility = {}
        for i, (r, c) in enumerate(points):
            br, bc = int(r // width), int(c // width)
            candidates = []
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    for j in buckets.get((br + dr, bc + dc), []):
                        if j != i:
                            distance = self._distance(points[i], points[j])
                            if distance <= width:
                                candidates.append((distance, j))
            candidates.sort()
            connected = 0
            for _, j in candidates:
                key = (min(i, j), max(i, j))
                if key not in visibility:
                    visibility[key] = line_of_sight(self.grid, self.rows, self.cols, points[i], points[j])
                if visibility[key]:
                    road_map[i].add(j)
                    road_map[j].add(i)
                    connected += 1
                    if connected >= self.k_neighbors:
                        break
        return [sorted(neighbors) for neighbors in road_map], points

    def _dijkstra_search(self, road_map, points, start, goal):
        """在 roadmap 上用 Dijkstra 算法搜索最短路径"""
        # 找到 start 和 goal 在 points 中的索引
        try:
            start_idx = points.index(start)
            goal_idx = points.index(goal)
        except ValueError:
            return []   # 起终点不在采样点中，无法规划

        n = len(points)
        dist = [float('inf')] * n
        prev = [-1] * n
        dist[start_idx] = 0
        pq = [(0, start_idx)]

        while pq:
            d, u = heapq.heappop(pq)
            if d > dist[u]:
                continue
            if u == goal_idx:
                # 回溯路径
                path = []
                cur = goal_idx
                while cur != -1:
                    path.append(points[cur])
                    cur = prev[cur]
                return path[::-1]   # 从起点到终点
            for v in road_map[u]:
                w = self._distance(points[u], points[v])
                if dist[u] + w < dist[v]:
                    dist[v] = dist[u] + w
                    prev[v] = u
                    heapq.heappush(pq, (dist[v], v))
        return []   # 无法到达

    def plan(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Path:
        # 确保起点和终点是自由栅格
        """采样自由点并建立无向路网，通过 Dijkstra 搜索起终点连接，失败返回空路径。"""
        if not (self.grid[start[0], start[1]] == 0 and self.grid[goal[0], goal[1]] == 0):
            return []
        self.diagnostics = {"attempts": []}
        initial = self.num_samples
        try:
            for count in dict.fromkeys([initial, max(initial, self.retry_num_samples)]):
                self.num_samples = count
                points = self._sample_free_points(start, goal)
                road_map, points = self._build_roadmap(points)
                path = self._dijkstra_search(road_map, points, start, goal)
                self.diagnostics["attempts"].append(dict(requested_samples=count,
                    actual_points=len(points), found=bool(path), k_neighbors=self.k_neighbors,
                    max_edge_len=self.max_edge_len))
                if path:
                    return path
            return []
        finally:
            self.num_samples = initial
