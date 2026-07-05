from .planners.a_star import AStarPlanner
from .planners.dijkstra import DijkstraPlanner
from .planners.prm import PRMPlanner
from .planners.rrt import RRTPlanner
from .planners.rrt_star import RRTStarPlanner

_registry = {
    'astar': AStarPlanner,
    'dijkstra': DijkstraPlanner,
    'prm': PRMPlanner,
    'rrt': RRTPlanner,
    'rrt_star': RRTStarPlanner,
}

def create_planner(name, grid):
    if name not in _registry:
        raise ValueError(f"未知算法: {name}，可选: {list(_registry.keys())}")
    return _registry[name](grid)