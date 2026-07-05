#!/usr/bin/env python3
import os
import sys
import yaml
import numpy as np
import subprocess

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(BASE_DIR)
sys.path.insert(0, PROJECT_ROOT)

from map_pipeline.src.grid_map import GridMap
from src.planner_base import Path
from src.planner_factory import create_planner
from src.algorithm_selector import select_algorithms
from src.comparator import compare_paths
from src.path_visualizer import draw_single_path, draw_comparison
from src.path_saver import save_final_path


def open_image(filepath):
    """非阻塞打开图片，立即返回"""
    try:
        if os.name == 'nt':
            os.startfile(filepath)
        else:
            subprocess.Popen(['xdg-open', filepath])
    except Exception as e:
        print(f"（无法自动打开图片，请手动查看: {e}）")


def main():
    config_path = os.path.join(BASE_DIR, "config.yaml")
    with open(config_path, 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)

    cfg = config['path_planner']
    interactive = cfg.get('interactive', True)
    default_algo = cfg.get('default_algorithm', 'astar')
    available = cfg.get('available_algorithms', ['astar'])
    output_dir = os.path.join(BASE_DIR, "data", "output")
    os.makedirs(output_dir, exist_ok=True)

    input_map = os.path.join(BASE_DIR, "data", "input", "grid_map.npz")
    input_sg = os.path.join(BASE_DIR, "data", "input", "start_goal.txt")
    if not os.path.exists(input_map) or not os.path.exists(input_sg):
        print("错误：缺少 grid_map.npz 或 start_goal.txt，请先运行 map_pipeline 并将文件复制到 path_planner/data/input/")
        sys.exit(1)

    grid_map = GridMap.load(input_map)
    with open(input_sg, 'r') as f:
        lines = f.read().strip().split('\n')
        start_grid = tuple(map(int, lines[0].split()))
        goal_grid = tuple(map(int, lines[1].split()))

    grid = grid_map.grid
    print(f"地图尺寸: {grid.shape}, 起点: {start_grid}, 终点: {goal_grid}")

    if interactive:
        selected = select_algorithms(available)
        if not selected:
            print("未选择任何算法，退出。")
            return
    else:
        selected = [default_algo]

    print("\n正在计算路径，请稍候...")
    results = compare_paths(selected, grid, start_grid, goal_grid, create_planner)
    print("路径计算完成。\n")

    print("--- 路径计算结果 ---")
    for name, data in results.items():
        path = data['path']
        if path:
            print(f"{name}: 步数={data['steps']}, 耗时={data['time']:.4f}s")
        else:
            print(f"{name}: 未找到路径")

    if len(selected) == 1:
        algo = selected[0]
        path = results[algo]['path']
        if not path:
            print("无可行路径，退出。")
            return
        preview_file = os.path.join(output_dir, "planned_path.png")
        draw_single_path(grid, path, start_grid, goal_grid, preview_file, draw_grid=True)
        print(f"路径预览图已保存: {preview_file}")
        open_image(preview_file)   # 非阻塞打开

        confirm = input("确认使用此路径？[Y/n]: ").strip().lower()
        if confirm == '' or confirm == 'y':
            final_path = path
        else:
            print("已取消。")
            return
    else:
        comparison_file = os.path.join(output_dir, "comparison.png")
        draw_comparison(grid, results, start_grid, goal_grid, comparison_file, draw_grid=True)
        print(f"算法对比图已保存: {comparison_file}")
        open_image(comparison_file)

        print("请选择一个最终使用的路径:")
        for i, name in enumerate(selected, 1):
            status = "✓" if results[name]['path'] else "✗"
            print(f"  [{i}] {name} {status}")
        choice = input("输入编号: ").strip()
        try:
            idx = int(choice) - 1
            final_algo = selected[idx]
        except (ValueError, IndexError):
            print("无效选择，退出。")
            return
        if not results[final_algo]['path']:
            print("所选算法未找到路径。")
            return
        final_path = results[final_algo]['path']
        final_img = os.path.join(output_dir, "final_path.png")
        draw_single_path(grid, final_path, start_grid, goal_grid, final_img, draw_grid=True)
        open_image(final_img)

    path_file = os.path.join(output_dir, "path_data.npy")
    save_final_path(final_path, path_file)
    print(f"最终路径已保存: {path_file}")


if __name__ == "__main__":
    main()