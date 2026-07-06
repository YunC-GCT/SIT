#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
全流程入口：栅格化 → 路径规划 → ROS2 循迹
Ctrl+C 处理：主进程收到 SIGINT 时,不是直接 terminate 子进程(Win32 TerminateProcess
不会触发子进程的 KeyboardInterrupt),而是向子进程转发 Ctrl+C 事件,让子进程走自己的
信号处理 + finally 块执行 _stop_robot,确保小车先停再退出。
"""
import os
import sys
import subprocess
import shutil
import signal

IS_WINDOWS = sys.platform == 'win32'
current_process = None


def send_ctrl_c(proc):
    """把 Ctrl+C 转发给子进程,让子进程走自己的信号处理 + finally(包含 _stop_robot)。
    Windows 用 CTRL_C_EVENT 事件,POSIX 用 SIGINT。
    子进程必须用 CREATE_NEW_PROCESS_GROUP (Windows) 或 start_new_session=True (POSIX) 启动,
    否则信号会被主进程吸收,转不过去。
    """
    if proc is None:
        return
    try:
        if IS_WINDOWS:
            proc.send_signal(signal.CTRL_C_EVENT)
        else:
            proc.send_signal(signal.SIGINT)
    except (ValueError, OSError):
        # 子进程已经退出
        pass


def global_signal_handler(sig, frame):
    """主进程 SIGINT 处理:转发 Ctrl+C 给子进程,等其优雅退出(小车先停),再走主进程退出。
    超时兜底用 terminate/kill,保证不会留孤儿进程持续 publish cmd_vel。
    """
    global current_process
    print("\n[Main] 收到中断信号,转发 Ctrl+C 到子进程(让其先停车再退出)...")
    if current_process is not None and current_process.poll() is None:
        send_ctrl_c(current_process)
        try:
            current_process.wait(timeout=8)
            print("[Main] 子进程已优雅退出。")
        except subprocess.TimeoutExpired:
            print("[Main] 子进程未在 8s 内退出,强制 terminate...")
            current_process.terminate()
            try:
                current_process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                print("[Main] terminate 不响应,kill。")
                current_process.kill()
                current_process.wait(timeout=2)
    sys.exit(0)


def run_script(script_path):
    """以新进程组方式启动子进程,这样 Ctrl+C 能被转发过去(触发其 KeyboardInterrupt + finally)。
    用 Popen.stdout 同步读子进程输出,让 SIGINT handler 能在主线程跑。
    """
    global current_process
    try:
        popen_kwargs = {
            'stdout': subprocess.PIPE,
            'stderr': subprocess.STDOUT,
            'text': True,
            'bufsize': 1,
            'encoding': 'utf-8',
            'errors': 'replace',
        }
        if IS_WINDOWS:
            popen_kwargs['creationflags'] = subprocess.CREATE_NEW_PROCESS_GROUP
        else:
            popen_kwargs['start_new_session'] = True

        current_process = subprocess.Popen(
            [sys.executable, script_path],
            **popen_kwargs,
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
        print("\n[Main] 没有已栅格化的地图,现在开始处理新地图...")
        if not run_script(map_pipe):
            return
    else:
        maps = [d for d in os.listdir(maps_dir) if os.path.isdir(os.path.join(maps_dir, d))]
        print("\n当前已栅格化的地图:")
        for i, m in enumerate(maps, 1):
            print(f"  [{i}] {m}")
        print("输入地图编号选择地图,或直接按回车开始处理新地图。")
        choice = input("请输入编号(或回车): ").strip()
        if choice:
            try:
                idx = int(choice) - 1
                current_map = maps[idx]
                src_dir = os.path.join(maps_dir, current_map)
                planner_input = os.path.join(base_dir, 'path_planner', 'data', 'input')
                os.makedirs(planner_input, exist_ok=True)
                shutil.copy(os.path.join(src_dir, 'grid_map.npz'), planner_input)
                shutil.copy(os.path.join(src_dir, 'start_goal.txt'), planner_input)
                print(f"[Main] 已加载地图: {current_map}")
            except (ValueError, IndexError):
                print("[Main] 无效选择,退出。")
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
            print("[Main] 未找到路径文件,无法进行运动控制。")
            break

        print("\n" + "=" * 40)
        print("【阶段2】小车运动控制")
        print("=" * 40)
        if not run_script(follower):
            print("[Main] 运动控制结束或中断。")
            break

        print("\n任务完成。")
        # 原来这里的 'r' 分支引用不存在的 run_re_pick.py,直接去掉
        cmd = input("输入 'm' 切换地图,'q' 退出: ").strip().lower()
        if cmd in ('m', 'q'):
            break

    print("[Main] 程序退出。")


if __name__ == "__main__":
    main()