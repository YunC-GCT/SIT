"""
跨管线共享的文件路径与数据契约常量。
避免文件名硬编码分散在 main.py / run_planner.py / run_follower.py 中。
"""
import os
from typing import Tuple


# ── 输出文件基线（相对于各模块的 data 目录） ──
GRID_MAP_FILENAME = "grid_map.npz"
START_GOAL_FILENAME = "start_goal.txt"
PATH_DATA_FILENAME = "path_data.npy"

# ── 校验函数 ──
def validate_start_goal(filepath: str,
                         max_rows: int = 10_000,
                         max_cols: int = 10_000) -> Tuple[Tuple[int, int], Tuple[int, int]]:
    """
    读取并校验 start_goal.txt。
    期望格式：两行，每行 "row col"（非负整数）。

    :param filepath:  文件路径
    :param max_rows:  允许的最大行索引
    :param max_cols:  允许的最大列索引
    :return:          (start_grid, goal_grid) 各为 (row, col) 元组
    :raises ValueError: 格式不合法时给出友好报错
    """
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"起点/终点文件不存在: {filepath}")

    with open(filepath, 'r') as f:
        content = f.read().strip()

    if not content:
        raise ValueError(
            f"{START_GOAL_FILENAME} 为空，应包含两行：\n"
            f"  <起点 row> <起点 col>\n"
            f"  <终点 row> <终点 col>")

    lines = content.split('\n')
    if len(lines) != 2:
        raise ValueError(
            f"{START_GOAL_FILENAME} 应恰好两行（起点+终点），当前 {len(lines)} 行。\n"
            f"格式：\n"
            f"  <起点 row> <起点 col>\n"
            f"  <终点 row> <终点 col>")

    points = []
    for i, line in enumerate(lines):
        parts = line.strip().split()
        if len(parts) != 2:
            raise ValueError(
                f"{START_GOAL_FILENAME} 第 {i+1} 行格式错误，应为两个整数。\n"
                f"  当前内容: '{line}'\n"
                f"  期望格式: '<row> <col>'")

        try:
            r = int(parts[0])
            c = int(parts[1])
        except ValueError:
            raise ValueError(
                f"{START_GOAL_FILENAME} 第 {i+1} 行无法解析为整数。\n"
                f"  当前内容: '{line}'")

        if r < 0 or c < 0:
            raise ValueError(
                f"{START_GOAL_FILENAME} 第 {i+1} 行坐标不得为负数: ({r}, {c})")
        if r >= max_rows or c >= max_cols:
            raise ValueError(
                f"{START_GOAL_FILENAME} 第 {i+1} 行坐标 ({r}, {c}) 超出地图范围 "
                f"({max_rows}×{max_cols})")

        points.append((r, c))

    return points[0], points[1]
