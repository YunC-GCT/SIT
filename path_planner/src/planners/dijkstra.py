import heapq
from ..planner_base import GlobalPlanner

class DijkstraPlanner(GlobalPlanner):
    def plan(self, start, goal):
        """在四邻接空闲栅格上按累计代价搜索，返回最短路径或空列表。"""
        rows, cols = self.grid.shape
        neighbors = [(1,0),(-1,0),(0,1),(0,-1)]
        open_heap = []
        heapq.heappush(open_heap, (0, start))
        came_from = {}
        g = {start: 0}
        while open_heap:
            g_curr, current = heapq.heappop(open_heap)
            if current == goal:
                path = []
                node = current
                while node in came_from:
                    path.append(node)
                    node = came_from[node]
                path.append(start)
                path.reverse()
                return path
            for dx, dy in neighbors:
                nx, ny = current[0]+dx, current[1]+dy
                if 0 <= nx < rows and 0 <= ny < cols and self.grid[nx, ny] == 0:
                    tentative_g = g[current] + 1
                    if tentative_g < g.get((nx, ny), float('inf')):
                        came_from[(nx, ny)] = current
                        g[(nx, ny)] = tentative_g
                        heapq.heappush(open_heap, (tentative_g, (nx, ny)))
        return []
