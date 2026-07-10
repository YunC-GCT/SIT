"""
MapPipeline — map_pipeline 的统一深接口。

将 run_pipeline.py 中 100+ 行的 main() 调用分散在 6 个浅模块中的逻辑收敛到
一个 process() 调用，隐藏加载→二值化→栅格化→坐标映射→校验的全流程。

内部仍委托给现有函数（map_loader / binarizer / grid_map / coordinate_mapper），
但外部调用者不需要知道这些细节。
"""
import os
import sys
from typing import Tuple
import numpy as np

from .src.map_loader import load_image, binarize, create_grid_map
from .src.grid_map import GridMap
from .src.coordinate_mapper import map_pixel_to_grid

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(BASE_DIR))
from pipeline_config import validate_start_goal


class MapPipeline:
    """地图栅格化流水线的一站式入口。"""

    def __init__(self, config: dict):
        """
        :param config: map_pipeline/config.yaml 的字典内容
        """
        self.default_width = config.get('default_width', 500)
        self.threshold = config.get('threshold', 0)
        self.blur_ksize = config.get('blur_ksize', 5)
        self.invert = config.get('invert', False)
        self.obstacle_ratio = config.get('obstacle_ratio', 0.5)
        self.grid_size = config.get('grid_size', 0.3)

    def process(self, image_path: str, start_px: Tuple[int, int],
                goal_px: Tuple[int, int]) -> Tuple[GridMap, Tuple[int, int], Tuple[int, int]]:
        """
        一站式处理：图片 → 二值化 → 栅格化 → 坐标映射。

        :param image_path: 原始图片路径
        :param start_px:   起点像素坐标 (x, y)
        :param goal_px:    终点像素坐标 (x, y)
        :return:           (GridMap, start_grid, goal_grid)
        """
        img = load_image(image_path)
        binary = binarize(img, threshold=self.threshold,
                          blur_ksize=self.blur_ksize, invert=self.invert)

        img_h, img_w = binary.shape
        cell_w = img_w / self.default_width
        grid_cols = self.default_width
        grid_rows = max(1, int(img_h / cell_w))

        grid_map, _ = create_grid_map(binary, self.default_width,
                                       obstacle_ratio=self.obstacle_ratio,
                                       grid_size=self.grid_size)

        start_grid = map_pixel_to_grid(start_px[0], start_px[1], grid_map)
        goal_grid = map_pixel_to_grid(goal_px[0], goal_px[1], grid_map)

        # 自动调整到空闲格
        if not grid_map.is_free(*start_grid):
            start_grid = grid_map.find_nearest_free(*start_grid)
        if not grid_map.is_free(*goal_grid):
            goal_grid = grid_map.find_nearest_free(*goal_grid)

        return grid_map, start_grid, goal_grid

    def save_outputs(self, grid_map: GridMap, start_grid: Tuple[int, int],
                     goal_grid: Tuple[int, int], output_dir: str):
        """保存栅格地图 + 起终点到指定目录。"""
        os.makedirs(output_dir, exist_ok=True)
        grid_map.save(os.path.join(output_dir, 'grid_map.npz'))
        with open(os.path.join(output_dir, 'start_goal.txt'), 'w') as f:
            f.write(f"{start_grid[0]} {start_grid[1]}\n{goal_grid[0]} {goal_grid[1]}\n")
