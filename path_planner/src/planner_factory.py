"""
算法注册表 + 工厂函数。

新增算法只需两步：
  1. 在 planners/ 下新建文件，实现 GlobalPlanner 接口
  2. 在下方 _registry 加一行注册（含默认参数）

config.yaml 的 algorithm_params 可以按需覆盖默认值；
available_algorithms 控制终端可选列表。
"""
from numbers import Integral, Real

from .planners.a_star import AStarPlanner
from .planners.dijkstra import DijkstraPlanner
from .planners.prm import PRMPlanner
from .planners.rrt import RRTPlanner
from .planners.rrt_adaptive import RRTAdaptivePlanner
from .planners.rrt_star import RRTStarPlanner

# 注册表：(PlannerClass, default_kwargs)
# default_kwargs 是算法的出厂默认参数，config.yaml 的 algorithm_params 可覆盖
_registry = {
    'astar':      (AStarPlanner,    {}),
    'dijkstra':   (DijkstraPlanner, {}),
    'prm':        (PRMPlanner,      {'num_samples': 500, 'k_neighbors': 10, 'max_edge_len': 30.0, 'retry_num_samples': 0}),
    'rrt':        (RRTPlanner,      {'max_iter': 2000, 'step_size': 5.0, 'goal_sample_rate': 0.1}),
    'rrt_adaptive': (RRTAdaptivePlanner, {'max_iter': 2000, 'step_size': 5.0, 'goal_sample_rate': 0.1, 'adaptive_step': True}),
    'rrt_star':   (RRTStarPlanner,  {'max_iter': 2000, 'step_size': 5.0,
                                      'goal_sample_rate': 0.1, 'search_radius': 10.0, 'adaptive_step': False,
                                      'optimize_iterations': None, 'search_time_s': 0.0, 'optimize_time_s': 0.0}),
}

RANDOM_ALGORITHMS = frozenset({"prm", "rrt", "rrt_adaptive", "rrt_star"})


class PlannerParameterError(ValueError):
    """规划器参数不符合统一参数契约。"""


def _is_number(value):
    """判断参数是否为有限实数，并排除布尔值。"""
    return isinstance(value, Real) and not isinstance(value, bool)


def validate_algorithm_parameters(name, params=None):
    """按算法的参数规则校验并规范化构造参数，未知或非法参数抛出参数错误。"""
    if name not in _registry:
        raise PlannerParameterError(
            f"未知算法: {name}，可选: {list(_registry.keys())}"
        )
    if params is None:
        return {}
    if not isinstance(params, dict):
        raise PlannerParameterError("算法参数必须是 JSON 对象")

    defaults = _registry[name][1]
    unknown = sorted(set(params) - set(defaults))
    if unknown:
        raise PlannerParameterError(
            f"算法 {name} 不支持参数: {', '.join(unknown)}"
        )

    normalized = dict(params)
    if "adaptive_step" in normalized and not isinstance(normalized["adaptive_step"], bool):
        raise PlannerParameterError("adaptive_step 必须是布尔值")
    for field in ("retry_num_samples", "optimize_iterations"):
        if field in normalized:
            value = normalized[field]
            if field == "optimize_iterations" and value is None:
                continue
            if not isinstance(value, Integral) or isinstance(value, bool) or value < 0:
                raise PlannerParameterError(f"{field} 必须是非负整数")
    for field in ("search_time_s", "optimize_time_s"):
        if field in normalized and (not _is_number(normalized[field]) or not 0 <= normalized[field] < float('inf')):
            raise PlannerParameterError(f"{field} 必须是有限非负数")
    integer_fields = {"num_samples", "k_neighbors", "max_iter"}
    positive_number_fields = {"max_edge_len", "step_size", "search_radius"}
    for field in integer_fields:
        if field in normalized and (
            not isinstance(normalized[field], Integral)
            or isinstance(normalized[field], bool)
            or normalized[field] <= 0
        ):
            raise PlannerParameterError(f"{field} 必须是正整数")
    for field in positive_number_fields:
        if field in normalized and (
            not _is_number(normalized[field]) or not 0 < normalized[field] < float('inf')
        ):
            raise PlannerParameterError(f"{field} 必须是正数")
    if "goal_sample_rate" in normalized and (
        not _is_number(normalized["goal_sample_rate"])
        or not 0 <= normalized["goal_sample_rate"] <= 1
    ):
        raise PlannerParameterError("goal_sample_rate 必须在 0 到 1 之间")
    return normalized


def resolve_algorithm_parameters(name, params=None):
    """合并已注册默认值与覆盖参数，返回经校验的实际算法配置。"""
    normalized = validate_algorithm_parameters(name, params)
    defaults = _registry[name][1]
    effective = dict(defaults)
    effective.update(normalized)
    return effective


def create_planner(name, grid, params=None, seed=None):
    """
    根据算法名创建规划器实例。

    :param name:   算法名（_registry 的 key）
    :param grid:   栅格地图 (ndarray, 0=空闲 1=障碍)
    :param params: 可选参数字典，会覆盖该算法的默认参数
    :return:       GlobalPlanner 实例
    """
    if params is None:
        raw_params = {}
    elif not isinstance(params, dict):
        raise PlannerParameterError("算法参数必须是 JSON 对象")
    else:
        raw_params = dict(params)
    if "seed" in raw_params:
        if seed is not None:
            raise PlannerParameterError("seed 只能通过 seed 参数或 params.seed 提供一次")
        seed = raw_params.pop("seed")
    if seed is not None and (
        not isinstance(seed, Integral) or isinstance(seed, bool)
    ):
        raise PlannerParameterError("seed 必须是整数或 null")
    normalized = resolve_algorithm_parameters(name, raw_params)
    planner_cls, defaults = _registry[name]
    merged = dict(defaults)
    merged.update(normalized)
    return planner_cls(grid, seed=None if seed is None else int(seed), **merged)


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
