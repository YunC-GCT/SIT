from __future__ import annotations

from abc import ABC, abstractmethod
import random
from typing import List, Tuple
import numpy as np

Path = List[Tuple[int, int]]

class GlobalPlanner(ABC):
    def __init__(self, grid: np.ndarray, seed: int | None = None):
        self.grid = np.asarray(grid)
        self.seed = seed
        self.rng = random.Random(seed)

    @abstractmethod
    def plan(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Path:
        pass
