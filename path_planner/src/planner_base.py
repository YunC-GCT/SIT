from abc import ABC, abstractmethod
from typing import List, Tuple
import numpy as np

Path = List[Tuple[int, int]]

class GlobalPlanner(ABC):
    def __init__(self, grid: np.ndarray):
        self.grid = grid

    @abstractmethod
    def plan(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Path:
        pass