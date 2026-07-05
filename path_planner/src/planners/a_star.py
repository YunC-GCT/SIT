import heapq
from ..planner_base import GlobalPlanner, Path

def manhattan_distance(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

class AStarPlanner(GlobalPlanner):
    def plan(self, start, goal):
        rows, cols = self.grid.shape
        neighbors = [(1,0),(-1,0),(0,1),(0,-1)]
        open_heap = []
        heapq.heappush(open_heap, (0 + manhattan_distance(start, goal), 0, start))
        came_from = {}
        g = {start: 0}
        while open_heap:
            f, g_curr, current = heapq.heappop(open_heap)
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
                        f_val = tentative_g + manhattan_distance((nx, ny), goal)
                        heapq.heappush(open_heap, (f_val, tentative_g, (nx, ny)))
        return []