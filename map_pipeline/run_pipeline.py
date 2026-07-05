#!/usr/bin/env python3
"""
地图处理管线入口：选点 → 调栅格尺寸 → 栅格化 → 输出标准文件 + 预览图
支持中文路径保存图片
"""

import os
import sys
import yaml
import cv2
import numpy as np
from src.map_loader import load_image, binarize, create_grid_map
from src.interactive_picker import pick_points
from src.coordinate_mapper import map_pixel_to_grid
from src.grid_resolution_tuner import tune_resolution
from src.grid_map import GridMap


def safe_imwrite(filepath, img):
    """支持中文路径的图片保存"""
    try:
        # 首先尝试 cv2.imwrite（部分新版本已支持中文）
        success = cv2.imwrite(filepath, img)
        if not success:
            raise Exception("cv2.imwrite 返回 False")
    except:
        # 失败时使用 imencode + 二进制写入
        ext = os.path.splitext(filepath)[1]
        success, encoded = cv2.imencode(ext, img)
        if success:
            with open(filepath, 'wb') as f:
                f.write(encoded.tobytes())
        else:
            print(f"  警告: 无法保存图片 {filepath}")


def draw_grid_preview(grid_map, start_grid, goal_grid, save_path, cell_size=4, draw_grid_lines=True, show_window=True):
    """
    绘制栅格地图预览图，并标记起终点。
    绘制后既保存到文件,也用 cv2.imshow 显示一个预览窗口（与 interactive_picker 同样的方式）,
    用户按任意键关闭后继续。
    :param grid_map: 二维数组 (0空闲, 1障碍)
    :param start_grid: (row, col)
    :param goal_grid: (row, col)
    :param save_path: 保存路径
    :param cell_size: 每个格子的像素大小，默认4
    :param draw_grid_lines: 是否绘制栅格线，默认 True
    :param show_window: 是否弹出预览窗口，默认 True
    """
    rows, cols = grid_map.shape
    img_height = rows * cell_size
    img_width = cols * cell_size
    # 白色背景
    img = np.ones((img_height, img_width, 3), dtype=np.uint8) * 255

    # 画障碍格（黑色）
    for r in range(rows):
        for c in range(cols):
            if grid_map[r, c] == 1:
                y1, y2 = r * cell_size, (r + 1) * cell_size
                x1, x2 = c * cell_size, (c + 1) * cell_size
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 0), -1)

    # 画栅格线（浅灰色细线）
    if draw_grid_lines:
        for r in range(1, rows):
            y = r * cell_size
            cv2.line(img, (0, y), (img_width, y), (200, 200, 200), 1)
        for c in range(1, cols):
            x = c * cell_size
            cv2.line(img, (x, 0), (x, img_height), (200, 200, 200), 1)

    # 起点（绿色）
    sr, sc = start_grid
    cy_start = int((sr + 0.5) * cell_size)
    cx_start = int((sc + 0.5) * cell_size)
    cv2.circle(img, (cx_start, cy_start), max(3, cell_size // 2), (0, 255, 0), -1)

    # 终点（红色）
    gr, gc = goal_grid
    cy_goal = int((gr + 0.5) * cell_size)
    cx_goal = int((gc + 0.5) * cell_size)
    cv2.circle(img, (cx_goal, cy_goal), max(3, cell_size // 2), (0, 0, 255), -1)

    # 保存文件
    safe_imwrite(save_path, img)

    # 弹出预览窗口（与 interactive_picker 同样的方式）
    if show_window:
        window_name = "Grid Preview (any key to close)"
        max_w, max_h = 1000, 700
        scale = min(max_w / img_width, max_h / img_height, 1.0)
        disp_w, disp_h = max(1, int(img_width * scale)), max(1, int(img_height * scale))
        display_img = cv2.resize(img, (disp_w, disp_h))
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_EXPANDED)
        cv2.resizeWindow(window_name, disp_w, disp_h)
        cv2.putText(display_img, "Grid Preview (press any key to close)",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.putText(display_img, "Green=Start  Red=Goal",
                    (10, disp_h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.imshow(window_name, display_img)
        print("\n栅格预览窗口已弹出，按任意键关闭...")
        cv2.waitKey(0)
        cv2.destroyWindow(window_name)


def main():
    # 路径处理
    base_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(base_dir, "config.yaml")
    with open(config_path, 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)

    # 数据目录
    input_dir = os.path.join(base_dir, "data", "input")
    output_dir = os.path.join(base_dir, "data", "output")
    os.makedirs(input_dir, exist_ok=True)
    os.makedirs(output_dir, exist_ok=True)

    # 1. 选择图片（默认取 input 下第一张）
    exts = ('.jpg', '.jpeg', '.png', '.bmp')
    files = sorted([f for f in os.listdir(input_dir) if f.lower().endswith(exts)])
    if not files:
        print("data/input 目录下无图片，请放入图片后重试。")
        sys.exit(1)
    img_path = os.path.join(input_dir, files[0])
    map_name = os.path.splitext(files[0])[0]
    print(f"使用图片: {files[0]}")

    # 2. 加载原图
    img = load_image(img_path)
    img_h, img_w = img.shape[:2]
    print(f"图片尺寸: {img_w} x {img_h}")

    # 3. 原图选点（仅像素坐标）
    start_px, goal_px = pick_points(img.copy())
    print(f"起点像素: {start_px}, 终点像素: {goal_px}")

    # 4. 交互调整栅格宽度
    def preview(width):
        cell = img_w / width
        rows = max(1, int(img_h / cell))
        return rows, width

    target_width = tune_resolution(config['default_width'], img_w, img_h, preview)

    # 5. 正式二值化与栅格化
    threshold = config.get('threshold', 0)
    blur_ksize = config.get('blur_ksize', 5)
    invert = config.get('invert', False)
    obstacle_ratio = config.get('obstacle_ratio', 0.5)

    binary = binarize(img, threshold=threshold, blur_ksize=blur_ksize, invert=invert)
    grid_map, _ = create_grid_map(binary, target_width, obstacle_ratio=obstacle_ratio)

    # 6. 坐标映射
    start_grid = map_pixel_to_grid(start_px[0], start_px[1], grid_map)
    goal_grid = map_pixel_to_grid(goal_px[0], goal_px[1], grid_map)
    print(f"栅格起点: {start_grid}, 终点: {goal_grid}")

    # 7. 校验并自动调整到空闲格
    if not grid_map.is_free(*start_grid):
        new_start = grid_map.find_nearest_free(*start_grid)
        print(f"起点为障碍，自动调整至: {new_start}")
        start_grid = new_start
    if not grid_map.is_free(*goal_grid):
        new_goal = grid_map.find_nearest_free(*goal_grid)
        print(f"终点为障碍，自动调整至: {new_goal}")
        goal_grid = new_goal

    # 8. 创建地图专属目录，保存所有文件
    map_dir = os.path.join(output_dir, "maps", map_name)
    os.makedirs(map_dir, exist_ok=True)

    # 8.1 保存栅格地图
    map_output = os.path.join(map_dir, "grid_map.npz")
    grid_map.save(map_output)

    # 8.2 保存起终点
    with open(os.path.join(map_dir, "start_goal.txt"), 'w') as f:
        f.write(f"{start_grid[0]} {start_grid[1]}\n{goal_grid[0]} {goal_grid[1]}\n")

    # 8.3 保存原图备份（使用安全写入）
    safe_imwrite(os.path.join(map_dir, "original.png"), img)

    # 8.4 保存该地图的配置参数
    map_config = {
        'threshold': threshold,
        'obstacle_ratio': obstacle_ratio,
        'target_width': target_width,
        'invert': invert,
        'blur_ksize': blur_ksize,
    }
    with open(os.path.join(map_dir, "config.yaml"), 'w') as f:
        yaml.dump(map_config, f)

    # 8.5 生成并保存栅格预览图
    preview_path = os.path.join(map_dir, "grid_preview.png")
    draw_grid_preview(grid_map.grid, start_grid, goal_grid, preview_path, cell_size=4)

    # 9. 完成提示
    print(f"\n地图 '{map_name}' 处理完成！")
    print(f"  栅格地图: {map_output}")
    print(f"  起终点  : {os.path.join(map_dir, 'start_goal.txt')}")
    print(f"  原图备份: {os.path.join(map_dir, 'original.png')}")
    print(f"  参数记录: {os.path.join(map_dir, 'config.yaml')}")
    print(f"  预览图  : {preview_path}")


if __name__ == "__main__":
    main()