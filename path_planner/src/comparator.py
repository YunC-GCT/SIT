import time
import numpy as np

def compare_paths(planner_names, grid, start, goal, factory_func):
    results = {}
    for name in planner_names:
        planner = factory_func(name, grid)
        t_start = time.perf_counter()
        path = planner.plan(start, goal)
        elapsed = time.perf_counter() - t_start
        steps = len(path) - 1 if path else None
        results[name] = {'path': path, 'steps': steps, 'time': elapsed}
    return results