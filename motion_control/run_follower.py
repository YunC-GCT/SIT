#!/usr/bin/env python3
import os
import sys
import yaml
import numpy as np
import signal
from src.ros2_interface import Ros2PathExecutor, delta_to_dir, compute_vel_commands


def interpolate_path(path):
    """
    把任意步长的路径节点展开为逐格路径（Bresenham 线段插值）。
    解决 PRM/RRT/RRT* 等采样算法产出大步长节点时，delta_to_dir()
    无法处理非 ±1 位移的问题。
    """
    if len(path) < 2:
        return path
    cells = [tuple(path[0])]
    for i in range(1, len(path)):
        r1, c1 = int(round(path[i-1][0])), int(round(path[i-1][1]))
        r2, c2 = int(round(path[i][0])),   int(round(path[i][1]))
        dr = abs(r2 - r1); dc = abs(c2 - c1)
        sr = 1 if r1 < r2 else -1; sc = 1 if c1 < c2 else -1
        err = dr - dc; r, c = r1, c1
        while r != r2 or c != c2:
            e2 = 2 * err
            if e2 > -dc: err -= dc; r += sr
            if e2 < dr:  err += dr; c += sc
            cells.append((r, c))
    return cells

# 自定义中断处理（仅用于本脚本被单独运行时）
def signal_handler(sig, frame):
    print("\n[Follower] 手动中断，程序即将退出（小车将在内部停止）")
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, signal_handler)

    base_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(base_dir, "config.yaml")
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    cfg = config['motion_control']
    mode = cfg.get('mode', 'ros2')
    path_file = os.path.join(base_dir, cfg.get('path_file', 'data/input/path_data.npy'))

    if not os.path.exists(path_file):
        print("[Follower] 路径文件不存在:", path_file)
        sys.exit(1)

    # 加载路径，展开为逐格步长（兼容 PRM/RRT/RRT* 的大步长输出）
    path = np.load(path_file)
    path = interpolate_path(path)
    deltas = [(int(path[i+1][0]) - int(path[i][0]), int(path[i+1][1]) - int(path[i][1]))
              for i in range(len(path)-1)]
    directions = [delta_to_dir(dr, dc) for dr, dc in deltas]
    initial_heading = cfg.get('initial_heading', 'east')
    move_speed = cfg['move_speed']
    vel_commands = compute_vel_commands(directions, initial_heading=initial_heading,
                                        move_speed=move_speed)

    print(f"[Follower] 路径段数: {len(deltas)}")
    print(f"[Follower] 方向序列: {directions}")
    print(f"[Follower] 麦轮速度指令 (vx, vy): {vel_commands}")

    # 从 grid_map.npz 读取真实 grid_size，与 config 校验
    grid_map_path = os.path.join(base_dir, 'data', 'input', 'grid_map.npz')
    config_grid_size = cfg['grid_size']
    if os.path.exists(grid_map_path):
        try:
            grid_meta = dict(np.load(grid_map_path, allow_pickle=True))
            map_grid_size = float(grid_meta.get('grid_size', config_grid_size))
            if abs(map_grid_size - config_grid_size) > 0.001:
                print(f"[Follower] ⚠️ grid_size 不一致！"
                      f"地图生成时记录: {map_grid_size:.4f}m, "
                      f"config.yaml 配置: {config_grid_size:.4f}m。"
                      f"将使用地图记录值 {map_grid_size:.4f}m。")
                cfg['grid_size'] = map_grid_size
            else:
                print(f"[Follower] grid_size 校验通过: {map_grid_size:.4f}m")
        except Exception as e:
            print(f"[Follower] 读取 grid_map.npz 失败({e})，使用 config 值")

    if mode == 'ros2':
        executor = Ros2PathExecutor(
            turn_speed=cfg['turn_speed'],
            move_speed=cfg['move_speed'],
            grid_size=cfg['grid_size'],
            timer_period=cfg['timer_period'],
            odom_topic=cfg.get('odom_topic', '/odom'),
            initial_heading=initial_heading,
        )
        try:
            executor.execute_path(deltas, vel_commands)
        except Exception as e:
            print(f"[Follower] 执行出错: {e}")
        # 注意：execute_path() 内部的 finally 已负责
        # _stop_robot() → destroy_node() → rclpy.shutdown()，
        # 此处不能再重复销毁，否则会二次销毁已释放的节点导致崩溃。
    else:
        print("[Follower] 仿真模式尚未实现")
        sys.exit(1)

if __name__ == "__main__":
    main()