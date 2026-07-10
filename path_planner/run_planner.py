#!/usr/bin/env python3
import os
import sys
import yaml
import numpy as np

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(BASE_DIR)
sys.path.insert(0, PROJECT_ROOT)

from map_pipeline.src.grid_map import GridMap
from src.planner_factory import create_planner
from src.algorithm_selector import select_algorithms
from src.comparator import compare_paths
from src.path_visualizer import draw_path, draw_comparison, show
from src.path_saver import save_final_path
from pipeline_config import validate_start_goal


def main():
    config_path = os.path.join(BASE_DIR, "config.yaml")
    with open(config_path, 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)

    cfg = config['path_planner']
    interactive = cfg.get('interactive', True)
    default_algo = cfg.get('default_algorithm', 'astar')
    available = cfg.get('available_algorithms', ['astar'])
    algo_params = cfg.get('algorithm_params', {})
    output_dir = os.path.join(BASE_DIR, "data", "output")
    os.makedirs(output_dir, exist_ok=True)

    input_map = os.path.join(BASE_DIR, "data", "input", "grid_map.npz")
    input_sg = os.path.join(BASE_DIR, "data", "input", "start_goal.txt")
    if not os.path.exists(input_map) or not os.path.exists(input_sg):
        print("错误：缺少 grid_map.npz 或 start_goal.txt")
        sys.exit(1)

    grid_map = GridMap.load(input_map)
    grid = grid_map.grid
    start_grid, goal_grid = validate_start_goal(input_sg,
                                                 max_rows=grid.shape[0],
                                                 max_cols=grid.shape[1])
    print(f"地图尺寸: {grid.shape}, 起点: {start_grid}, 终点: {goal_grid}")

    if interactive:
        selected = select_algorithms(available)
        if not selected:
            print("未选择任何算法，退出。")
            return
    else:
        selected = [default_algo]

    print("\n正在计算路径，请稍候...")
    results = compare_paths(selected, grid, start_grid, goal_grid,
                             create_planner, algo_params=algo_params)
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

        try:
            vis = draw_path(grid, path, start_grid, goal_grid)
            show("Planned Path", vis)
        except Exception as e:
            print(f"[Planner] cv2 预览不可用: {e}")

        confirm = input("确认使用此路径？[Y/n]: ").strip().lower()
        if confirm == '' or confirm == 'y':
            final_path = path
        else:
            print("已取消。")
            return
    else:
        try:
            vis = draw_comparison(grid, results, start_grid, goal_grid)
            show("Algorithm Comparison", vis)
        except Exception as e:
            print(f"[Planner] cv2 预览不可用: {e}")

        print("请选择一个最终使用的路径:")
        for i, name in enumerate(selected, 1):
            status = "v" if results[name]['path'] else "x"
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

        try:
            vis = draw_path(grid, final_path, start_grid, goal_grid)
            show("Final Path", vis)
        except Exception as e:
            print(f"[Planner] cv2 预览不可用: {e}")

    path_file = os.path.join(output_dir, "path_data.npy")
    save_final_path(final_path, path_file)
    print(f"最终路径已保存: {path_file}")


if __name__ == "__main__":
    main()
