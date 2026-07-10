"""
基于概率路图（PRM）的全局路径规划器（栅格版本）
直接在栅格地图上采样自由点、建立路图，并用 Dijkstra 搜索路径。
"""

import math
import random
import heapq
import numpy as np
from typing import List, Tuple
from ..planner_base import GlobalPlanner, Path
from ..grid_utils import line_of_sight

class PRMPlanner(GlobalPlanner):
    def __init__(self, grid: np.ndarray, num_samples: int = 500, k_neighbors: int = 10,
                 max_edge_len: float = 30.0):
        """
        :param grid: 栅格地图，0=空闲，1=障碍
        :param num_samples: 随机采样点数量（不含起终点）
        :param k_neighbors: 每个点最多连接的近邻数
        :param max_edge_len: 单条边的最大长度（栅格单元数），超过则不连接
        """
        super().__init__(grid)
        self.num_samples = num_samples
        self.k_neighbors = k_neighbors
        self.max_edge_len = max_edge_len
        self.rows, self.cols = grid.shape

    def _sample_free_points(self, start, goal):
        """在自由栅格中随机采样，并添加起终点"""
        points = []
        # 随机采样
        for _ in range(self.num_samples):
            r = random.randint(0, self.rows - 1)
            c = random.randint(0, self.cols - 1)
            if self.grid[r, c] == 0:
                points.append((r, c))
        # 确保起终点在集合中（如果它们是自由的话）
        if self.grid[start] == 0:
            points.append(start)
        if self.grid[goal] == 0:
            points.append(goal)
        # 去重（起终点可能与随机点重复）
        points = list(set(points))
        return points

    def _distance(self, p1, p2):
        """两点之间的欧氏距离（栅格坐标）"""
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

    def _build_roadmap(self, points):
        """
        为所有采样点建立邻接表形式的 roadmap。
        :return: road_map (每个点的邻居列表索引), points (最终使用的点列表)
        """
        n = len(points)
        road_map = [[] for _ in range(n)]
        # 用简单的暴力搜索来查找每个点的 K 近邻（采样点数量不大时可行）
        for i in range(n):
            # 计算点 i 到所有其他点的距离，按距离排序
            dists = [(self._distance(points[i], points[j]), j) for j in range(n) if i != j]
            dists.sort(key=lambda x: x[0])
            # 检查前 k_neighbors 个最近的邻居
            for d, j in dists[:self.k_neighbors]:
                if d > self.max_edge_len:
                    break   # 太远的不连接
                if line_of_sight(self.grid, self.rows, self.cols, points[i], points[j]):
                    road_map[i].append(j)
                    road_map[j].append(i)  # 无向图
        return road_map, points

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
        if not (self.grid[start[0], start[1]] == 0 and self.grid[goal[0], goal[1]] == 0):
            return []
        points = self._sample_free_points(start, goal)
        if len(points) < 2:
            return []
        road_map, points = self._build_roadmap(points)
        path = self._dijkstra_search(road_map, points, start, goal)
        return path
    