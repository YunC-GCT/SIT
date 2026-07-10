"""
算法注册表 + 工厂函数。

新增算法只需两步：
  1. 在 planners/ 下新建文件，实现 GlobalPlanner 接口
  2. 在下方 _registry 加一行注册（含默认参数）

config.yaml 的 algorithm_params 可以按需覆盖默认值；
available_algorithms 控制终端可选列表。
"""
from .planners.a_star import AStarPlanner
from .planners.dijkstra import DijkstraPlanner
from .planners.prm import PRMPlanner
from .planners.rrt import RRTPlanner
from .planners.rrt_star import RRTStarPlanner

# 注册表：(PlannerClass, default_kwargs)
# default_kwargs 是算法的出厂默认参数，config.yaml 的 algorithm_params 可覆盖
_registry = {
    'astar':      (AStarPlanner,    {}),
    'dijkstra':   (DijkstraPlanner, {}),
    'prm':        (PRMPlanner,      {'num_samples': 500, 'k_neighbors': 10, 'max_edge_len': 30.0}),
    'rrt':        (RRTPlanner,      {'max_iter': 2000, 'step_size': 5.0, 'goal_sample_rate': 0.1}),
    'rrt_star':   (RRTStarPlanner,  {'max_iter': 2000, 'step_size': 5.0,
                                      'goal_sample_rate': 0.1, 'search_radius': 10.0}),
}


def create_planner(name, grid, params=None):
    """
    根据算法名创建规划器实例。

    :param name:   算法名（_registry 的 key）
    :param grid:   栅格地图 (ndarray, 0=空闲 1=障碍)
    :param params: 可选参数字典，会覆盖该算法的默认参数
    :return:       GlobalPlanner 实例
    """
    if name not in _registry:
        raise ValueError(f"未知算法: {name}，可选: {list(_registry.keys())}")
    planner_cls, defaults = _registry[name]
    merged = dict(defaults)
    if params:
        merged.update(params)
    return planner_cls(grid, **merged)


def list_algorithms():
    """返回所有已注册的算法名称列表。"""
    return list(_registry.keys())


def get_algorithm_info(name):
    """
    返回算法的简要信息，便于调试/文档。
    :return: (class_name, default_params_dict)
    """
    if name not in _registry:
        raise ValueError(f"未知算法: {name}")
    planner_cls, defaults = _registry[name]
    return planner_cls.__name__, dict(defaults)
