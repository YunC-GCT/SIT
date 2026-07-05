#!/usr/bin/env python3
import os
import sys
import yaml
import numpy as np
import signal
from src.ros2_interface import Ros2PathExecutor, delta_to_dir, compute_turn_angles

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

    # 加载路径
    path = np.load(path_file)
    deltas = [(path[i+1][0] - path[i][0], path[i+1][1] - path[i][1]) for i in range(len(path)-1)]
    directions = [delta_to_dir(dr, dc) for dr, dc in deltas]
    turn_angles = compute_turn_angles(directions)

    print(f"[Follower] 路径段数: {len(deltas)}")
    print(f"[Follower] 方向序列: {directions}")
    print(f"[Follower] 转向角: {turn_angles}")

    if mode == 'ros2':
        executor = Ros2PathExecutor(
            turn_speed=cfg['turn_speed'],
            move_speed=cfg['move_speed'],
            grid_size=cfg['grid_size'],
            timer_period=cfg['timer_period']
        )
        try:
            executor.execute_path(deltas, turn_angles)
        except Exception as e:
            print(f"[Follower] 执行出错: {e}")
        finally:
            # 确保节点被清理
            if executor.node is not None:
                executor.node.destroy_node()
    else:
        print("[Follower] 仿真模式尚未实现")
        sys.exit(1)

if __name__ == "__main__":
    main()