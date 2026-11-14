from __future__ import annotations

from abc import ABC, abstractmethod
import random
from typing import List, Tuple
import numpy as np

Path = List[Tuple[int, int]]

class GlobalPlanner(ABC):
    def __init__(self, grid: np.ndarray, seed: int | None = None):
        """保存栅格数组与随机种子，建立独立随机数生成器，供具体规划算法使用。"""
        self.grid = np.asarray(grid)
        self.seed = seed
        self.rng = random.Random(seed)

    @abstractmethod
    def plan(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Path:
        """定义规划接口：输入 row,col 起终点，返回路径点列表，搜索失败时返回空列表。"""
        pass
