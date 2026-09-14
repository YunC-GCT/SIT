"""
MapPipeline — map_pipeline 的统一深接口。

将 run_pipeline.py 中 100+ 行的 main() 调用分散在 6 个浅模块中的逻辑收敛到
一个 process() 调用，隐藏加载→二值化→栅格化→坐标映射→校验的全流程。

内部仍委托给现有函数（map_loader / binarizer / grid_map / coordinate_mapper），
但外部调用者不需要知道这些细节。
"""
import os
from typing import Tuple

from .src.map_loader import load_image, binarize, create_grid_map
from .src.grid_map import GridMap
from .src.coordinate_mapper import map_pixel_to_grid, validate_grid_points
from core import grid_map_payload, make_result, write_result

BASE_DIR = os.path.dirname(os.path.abspath(__file__))


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

        for name, point in (("start", start_px), ("goal", goal_px)):
            x, y = point
            if not (0 <= x < img.shape[1] and 0 <= y < img.shape[0]):
                raise ValueError(
                    f"{name} 像素坐标超出图片范围: ({x}, {y}), "
                    f"shape=({img.shape[0]},{img.shape[1]})"
                )

        grid_map, _ = create_grid_map(binary, self.default_width,
                                       obstacle_ratio=self.obstacle_ratio,
                                       grid_size=self.grid_size)

        start_grid = map_pixel_to_grid(start_px[0], start_px[1], grid_map)
        goal_grid = map_pixel_to_grid(goal_px[0], goal_px[1], grid_map)

        # 起终点必须由用户明确选在空闲格，不静默移动到其他位置。
        start_grid, goal_grid = validate_grid_points(
            grid_map,
            start_grid,
            goal_grid,
        )

        return grid_map, start_grid, goal_grid

    def save_outputs(self, grid_map: GridMap, start_grid: Tuple[int, int],
                     goal_grid: Tuple[int, int], output_dir: str,
                     run_id: str = "standalone"):
        """保存版本化地图 JSON 到指定目录。"""
        os.makedirs(output_dir, exist_ok=True)
        result_path = os.path.join(output_dir, "map_result.json")
        result = make_result(
            "map_pipeline",
            run_id,
            "success",
            outputs={
                "map": grid_map_payload(
                    grid_map.grid.astype(int).tolist(),
                    grid_size_m=float(grid_map.grid_size),
                    start=start_grid,
                    goal=goal_grid,
                )
            },
            parameters={
                "default_width": grid_map.cols,
                "grid_size": float(grid_map.grid_size),
            },
        )
        write_result(result_path, result)
        return result_path
