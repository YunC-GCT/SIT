"""Fair A* vs RRT* comparison — interpolate RRT* waypoints to grid cells."""
import sys, os, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cv2, numpy as np
from collections import deque
from path_planner.src.planners.a_star import AStarPlanner
from path_planner.src.planners.rrt_star import RRTStarPlanner

MAPS_DIR = os.path.join(os.path.dirname(__file__), "map_pipeline", "data", "input")
START = (3, 3)
GOAL  = (76, 76)


def load_grid(png_path):
    img = cv2.imdecode(np.fromfile(png_path, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
    if img is None: return None
    if img.shape != (80, 80):
        img = cv2.resize(img, (80, 80), interpolation=cv2.INTER_NEAREST)
    return (img < 128).astype(np.uint8)


def ensure_free(grid, pt):
    r, c = pt
    if grid[r, c] == 0: return pt
    q = deque([(r, c)])
    visited = {(r, c)}
    while q:
        cr, cc = q.popleft()
        for dr, dc in [(1,0),(-1,0),(0,1),(0,-1)]:
            nr, nc = cr+dr, cc+dc
            if 0 <= nr < 80 and 0 <= nc < 80 and (nr,nc) not in visited:
                if grid[nr,nc] == 0: return (nr,nc)
                visited.add((nr,nc)); q.append((nr,nc))
    return pt


def bresenham_cells(p1, p2):
    """Bresenham 走一遍，返回途经的所有栅格坐标列表（含起终点）。"""
    r1, c1 = int(round(p1[0])), int(round(p1[1]))
    r2, c2 = int(round(p2[0])), int(round(p2[1]))
    cells = [(r1, c1)]
    dr = abs(r2 - r1); dc = abs(c2 - c1)
    sr = 1 if r1 < r2 else -1; sc = 1 if c1 < c2 else -1
    err = dr - dc; r, c = r1, c1
    while r != r2 or c != c2:
        e2 = 2 * err
        if e2 > -dc: err -= dc; r += sr
        if e2 < dr:  err += dr; c += sc
        cells.append((r, c))
    return cells


def interpolate_path(waypoints):
    """把 RRT* 跳跃节点路径展开为逐格路径。"""
    if len(waypoints) < 2:
        return waypoints
    cells = [waypoints[0]]
    for i in range(1, len(waypoints)):
        seg = bresenham_cells(waypoints[i-1], waypoints[i])
        cells.extend(seg[1:])  # 跳过重复的起点
    return cells


print("=" * 78)
print(f"{'Map':<16} {'Algo':<7} {'Waypts':>7} {'GridCells':>10} {'Time':>8}  {'Straight':>9}")
print("-" * 78)

for fname in sorted(os.listdir(MAPS_DIR)):
    if not fname.endswith('.png'): continue
    grid = load_grid(os.path.join(MAPS_DIR, fname))
    if grid is None: continue

    s = ensure_free(grid, START)
    g = ensure_free(grid, GOAL)
    straight = np.sqrt((g[0]-s[0])**2 + (g[1]-s[1])**2)

    # A*
    t0 = time.perf_counter()
    p_a = AStarPlanner(grid).plan(s, g)
    t_a = time.perf_counter() - t0
    a_cells = len(p_a) - 1 if p_a else 0

    # RRT*
    t0 = time.perf_counter()
    p_r = RRTStarPlanner(grid, max_iter=3000, step_size=5,
                          goal_sample_rate=0.15, search_radius=15.0).plan(s, g)
    t_r = time.perf_counter() - t0
    r_waypts = len(p_r) - 1 if p_r else 0
    # 展开为逐格路径
    r_interp = interpolate_path(p_r) if p_r else []
    r_cells = len(r_interp) - 1 if r_interp else 0

    name = fname.replace('.png', '')
    print(f"{name:<16} {'A*':<7} {'--':>7} {a_cells:>10}  {t_a:>7.4f}s  {straight:>7.1f}")
    print(f"{'':16} {'RRT*':<7} {r_waypts:>7} {r_cells:>10}  {t_r:>7.4f}s")

print("-" * 78)
print("Waypts   = number of tree nodes in RRT* path (before interpolation)")
print("GridCells = actual grid steps after Bresenham interpolation")
print("Straight  = Euclidean distance start→goal (lower bound)")
