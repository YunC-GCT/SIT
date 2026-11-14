"""Public algorithm catalog used by CLI and terminal selection."""

from pathlib import Path

import yaml

from .src.planner_factory import list_algorithms


LABELS = {
    "astar": "A*（启发式搜索）",
    "dijkstra": "Dijkstra（最短路径搜索）",
    "prm": "PRM（概率路线图）",
    "rrt": "RRT（快速探索随机树）",
    "rrt_adaptive": "自适应 RRT（独立实验变体）",
    "rrt_star": "RRT*（重布线优化）",
}


def algorithm_catalog(config_path: str | None = None) -> tuple[list[dict[str, str]], str]:
    """Return configured registered algorithms and the configured default.

    Labels are descriptive only; canonical names remain the JSON/CLI contract.
    Registry additions appear without modifying the terminal chooser.
    """
    path = Path(config_path) if config_path else Path(__file__).with_name("config.yaml")
    with path.open(encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}
    config = raw.get("path_planner", raw)
    registered = list_algorithms()
    names = config.get("available_algorithms", registered)
    if not isinstance(names, list) or not names or any(
        not isinstance(name, str) or name not in registered for name in names
    ):
        raise ValueError("available_algorithms 必须是非空的已注册算法名称列表")
    default = config.get("default_algorithm", "astar")
    if not isinstance(default, str) or default not in names:
        raise ValueError("default_algorithm 必须在 available_algorithms 中")
    entries: list[dict[str, str]] = []
    for name in names:
        assert isinstance(name, str)
        if not any(item["name"] == name for item in entries):
            entries.append({"name": name, "label": LABELS.get(name, name)})
    return entries, default
