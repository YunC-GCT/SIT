"""Explicit RRT variant with clearance-aware extension and collision backoff."""
from __future__ import annotations

import numpy as np
from .rrt import RRTPlanner
from ..grid_utils import line_of_sight


class RRTAdaptivePlanner(RRTPlanner):
    def __init__(self, grid, max_iter=2000, step_size=5.0, goal_sample_rate=0.1,
                 seed=None, adaptive_step=True):
        super().__init__(grid, max_iter=max_iter, step_size=step_size,
                         goal_sample_rate=goal_sample_rate, seed=seed)
        self.adaptive_step = adaptive_step

    def _extension(self, nearest, sample):
        """Shrink blocked extensions, down to one grid cell; keep fixed-step mode available."""
        step = self.step_size
        if self.adaptive_step:
            r, c = nearest
            radius = max(1, int(self.step_size))
            r0, r1 = max(0, r-radius), min(self.rows, r+radius+1)
            c0, c1 = max(0, c-radius), min(self.cols, c+radius+1)
            obstacles = np.argwhere(self.grid[r0:r1, c0:c1] != 0)
            if len(obstacles):
                clearance = float(np.min(np.max(np.abs(obstacles + (r0, c0) - (r, c)), axis=1)))
                step = min(step, max(1.0, clearance * 0.5))
        distance = self._distance(nearest, sample)
        while True:
            ratio = min(1.0, step / distance) if distance else 0.0
            point = (int(round(nearest[0] + ratio*(sample[0]-nearest[0]))),
                     int(round(nearest[1] + ratio*(sample[1]-nearest[1]))))
            if line_of_sight(self.grid, self.rows, self.cols, nearest, point):
                return point
            if not self.adaptive_step or step <= 1.0:
                return None
            step = max(1.0, step * 0.5)
