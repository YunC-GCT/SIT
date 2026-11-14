"""方向码、航向角及相对转角的几何辅助函数。"""

from __future__ import annotations

import math
from typing import Sequence

DIR_NORTH, DIR_EAST, DIR_SOUTH, DIR_WEST = 0, 1, 2, 3

HEADING_TO_DIR = {
    "north": DIR_NORTH,
    "east": DIR_EAST,
    "south": DIR_SOUTH,
    "west": DIR_WEST,
    "n": DIR_NORTH,
    "e": DIR_EAST,
    "s": DIR_SOUTH,
    "w": DIR_WEST,
}
HEADING_TO_DEG = {
    "north": 0.0,
    "east": 90.0,
    "south": 180.0,
    "west": 270.0,
    "n": 0.0,
    "e": 90.0,
    "s": 180.0,
    "w": 270.0,
}

def _diff_to_turn(diff: int) -> int:
    """把四方向的模 4 差值转换为 0、90、180 或 -90 度转角。"""
    if diff == 0:
        return 0
    if diff == 1:
        return 90
    if diff == 2:
        return 180
    return -90

def delta_to_dir(dr: int, dc: int) -> int:
    """将轴对齐栅格位移映射为北、东、南、西方向码，其他情况沿用默认北向。"""
    if dr < 0 and dc == 0:
        return DIR_NORTH
    if dr == 0 and dc > 0:
        return DIR_EAST
    if dr > 0 and dc == 0:
        return DIR_SOUTH
    if dr == 0 and dc < 0:
        return DIR_WEST
    return DIR_NORTH

def compute_turn_angles(
    directions: Sequence[int],
    initial_heading: str | int = "north",
) -> list[int]:
    """根据四方向序列和初始朝向计算各步相对转角。"""
    if not directions:
        return []
    if isinstance(initial_heading, str):
        try:
            current = HEADING_TO_DIR[initial_heading.lower()]
        except KeyError as exc:
            raise ValueError(
                "initial_heading must be north, east, south, west, or a direction code"
            ) from exc
    else:
        current = int(initial_heading)
    angles = []
    for target in directions:
        angles.append(_diff_to_turn((int(target) - current) % 4))
        current = int(target)
    return angles

def estimate_runtime(
    turn_angles: Sequence[float],
    grid_size: float,
    turn_speed: float,
    move_speed: float,
) -> float:
    """按每步一个栅格及给定速度估算移动和转向总时间，不包含通信或过渡开销。"""
    if not turn_angles:
        return 0.0
    turn_time = sum(math.radians(abs(angle)) for angle in turn_angles) / turn_speed
    move_time = len(turn_angles) * grid_size / move_speed
    return turn_time + move_time

def normalize_angle_deg(angle: float) -> float:
    """将角度归一到 (-180, 180] 度，统一把半周转角表示为正 180 度。"""
    normalized = (float(angle) + 180.0) % 360.0 - 180.0
    if math.isclose(normalized, -180.0, abs_tol=1e-9):
        return 180.0
    return normalized

def split_turn_angle_deg(angle: float) -> list[float]:
    """把半周旋转拆为两个同向 90 度动作；零角度返回空列表。"""
    angle = normalize_angle_deg(angle)
    if math.isclose(abs(angle), 180.0, abs_tol=1e-9):
        sign = 1.0 if angle >= 0 else -1.0
        return [90.0 * sign, 90.0 * sign]
    return [angle] if not math.isclose(angle, 0.0, abs_tol=1e-9) else []

def heading_degrees(value: str | float | int) -> float:
    """将方向名称或数值转换为北零度、顺时针递增的 [0, 360) 航向角。"""
    if isinstance(value, str):
        try:
            return HEADING_TO_DEG[value.lower()]
        except KeyError as exc:
            raise ValueError(
                "initial_heading must be north, east, south, west, or degrees"
            ) from exc
    return float(value) % 360.0

def compute_segment_turns(
    segments: Sequence[dict],
    *,
    initial_heading: str | float = "north",
) -> list[float]:
    """依据路径段 direction_deg 和初始朝向计算每段开始前的最短相对转角。"""
    current_heading = heading_degrees(initial_heading)
    turns = []
    for segment in segments:
        target = float(segment["direction_deg"]) % 360.0
        turn = normalize_angle_deg(target - current_heading)
        turns.append(turn)
        current_heading = target
    return turns


__all__ = [
    "DIR_EAST",
    "DIR_NORTH",
    "DIR_SOUTH",
    "DIR_WEST",
    "HEADING_TO_DEG",
    "HEADING_TO_DIR",
    "compute_segment_turns",
    "compute_turn_angles",
    "delta_to_dir",
    "estimate_runtime",
    "heading_degrees",
    "normalize_angle_deg",
    "split_turn_angle_deg",
]
