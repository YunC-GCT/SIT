"""
将原图像素坐标映射为栅格坐标
"""

from .grid_map import GridMap

def map_pixel_to_grid(pixel_x: int, pixel_y: int, grid_map: GridMap):
    """
    根据栅格地图的 cell 尺寸将像素坐标转换为栅格索引。
    pixel_x: 列方向 (对应 col)
    pixel_y: 行方向 (对应 row)
    """
    col = int(pixel_x / grid_map.cell_width)
    row = int(pixel_y / grid_map.cell_height)
    # 边界裁剪
    col = max(0, min(grid_map.cols - 1, col))
    row = max(0, min(grid_map.rows - 1, row))
    return row, col