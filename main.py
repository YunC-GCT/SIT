#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys
import subprocess
import shutil
import signal

current_process = None

def global_signal_handler(sig, frame):
    global current_process
    print("\n[Main] 收到中断信号，正在停止...")
    if current_process is not None:
        current_process.terminate()
        try:
            current_process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            current_process.kill()
        print("[Main] 子进程已终止。")
    sys.exit(0)

def run_script(script_path):
    global current_process
    try:
        current_process = subprocess.Popen(
            [sys.executable, script_path],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            encoding='utf-8',
            errors='replace'  # 防止乱码
        )
        for line in current_process.stdout:
            print(line, end='')
        current_process.wait()
        if current_process.returncode != 0:
            print(f"[Main] 脚本 {script_path} 退出码: {current_process.returncode}")
            return False
        return True
    except Exception as e:
        print(f"[Main] 运行脚本异常: {e}")
        return False
    finally:
        current_process = None

def main():
    signal.signal(signal.SIGINT, global_signal_handler)

    print("\n" + "=" * 60)
    print("      机器人路径规划与循迹系统")
    print("      支持多地图、五大算法、ROS2小车控制")
    print("=" * 60)

    base_dir = os.path.dirname(os.path.abspath(__file__))
    map_pipe = os.path.join(base_dir, 'map_pipeline', 'run_pipeline.py')
    planner = os.path.join(base_dir, 'path_planner', 'run_planner.py')
    follower = os.path.join(base_dir, 'motion_control', 'run_follower.py')
    maps_dir = os.path.join(base_dir, 'map_pipeline', 'data', 'output', 'maps')

    if not os.path.exists(maps_dir) or not os.listdir(maps_dir):
        print("\n[Main] 没有已栅格化的地图，现在开始处理新地图...")
        if not run_script(map_pipe):
            return
    else:
        maps = [d for d in os.listdir(maps_dir) if os.path.isdir(os.path.join(maps_dir, d))]
        print("\n当前已栅格化的地图：")
        for i, m in enumerate(maps, 1):
            print(f"  [{i}] {m}")
        print("输入地图编号选择地图，或直接按回车开始处理新地图。")
        choice = input("请输入编号（或回车）: ").strip()
        if choice:
            try:
                idx = int(choice) - 1
                current_map = maps[idx]
                src_dir = os.path.join(maps_dir, current_map)
                planner_input = os.path.join(base_dir, 'path_planner', 'data', 'input')
                os.makedirs(planner_input, exist_ok=True)
                shutil.copy(os.path.join(src_dir, 'grid_map.npz'), planner_input)
                shutil.copy(os.path.join(src_dir, 'start_goal.txt'), planner_input)
                print(f"[Main] 已加载地图：{current_map}")
            except (ValueError, IndexError):
                print("[Main] 无效选择，退出。")
                return
        else:
            if not run_script(map_pipe):
                return

    while True:
        print("\n" + "=" * 40)
        print("【阶段1】路径规划")
        print("=" * 40)
        if not run_script(planner):
            print("[Main] 路径规划失败或中断。")
            break

        path_src = os.path.join(base_dir, 'path_planner', 'data', 'output', 'path_data.npy')
        if os.path.exists(path_src):
            control_input = os.path.join(base_dir, 'motion_control', 'data', 'input')
            os.makedirs(control_input, exist_ok=True)
            shutil.copy(path_src, control_input)
        else:
            print("[Main] 未找到路径文件，无法进行运动控制。")
            break

        print("\n" + "=" * 40)
        print("【阶段2】小车运动控制")
        print("=" * 40)
        if not run_script(follower):
            print("[Main] 运动控制结束或中断。")
            break

        print("\n任务完成。")
        cmd = input("输入 'r' 重新选点，'m' 切换地图，'q' 退出: ").strip().lower()
        if cmd == 'r':
            re_pick = os.path.join(base_dir, 'map_pipeline', 'run_re_pick.py')
            if os.path.exists(re_pick):
                run_script(re_pick)
            else:
                print("重新选点功能尚未实现，请手动运行 map_pipeline 并重新选择地图。")
                break
        elif cmd == 'm':
            break
        elif cmd == 'q':
            break

    print("[Main] 程序退出。")

if __name__ == "__main__":
    main()