"""
多算法对比运行器。
"""
import time


def compare_paths(planner_names, grid, start, goal, factory_func, algo_params=None):
    """
    对多个算法逐一运行，返回结果字典。

    :param planner_names: 算法名列表
    :param grid:           栅格地图
    :param start:          起点 (row, col)
    :param goal:           终点 (row, col)
    :param factory_func:   工厂函数 create_planner(name, grid, params)
    :param algo_params:    可选 {name: {param: val}, ...}，覆盖算法默认参数
    :return:               {name: {'path': [...], 'steps': n, 'time': s}, ...}
    """
    if algo_params is None:
        algo_params = {}
    results = {}
    for name in planner_names:
        params = algo_params.get(name, None)
        planner = factory_func(name, grid, params=params)
        t_start = time.perf_counter()
        path = planner.plan(start, goal)
        elapsed = time.perf_counter() - t_start
        steps = len(path) - 1 if path else None
        results[name] = {'path': path, 'steps': steps, 'time': elapsed}
    return results
