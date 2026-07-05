SIT_LJGH_PROJECT —— 机器人路径规划与循迹系统
1. 项目概述
本项目实现了一个完整的机器人路径规划与自动控制流程，支持：

通过任意地图图片自动生成栅格地图

交互式选择起点和终点

多种全局路径规划算法（A*、Dijkstra、RRT、RRT*、PRM）

多算法对比与路径预览（含栅格网格线）

基于 ROS2 的树莓派麦克纳姆轮小车自动循迹控制

统一调度入口 main.py，支持地图复用、循环任务，并安全响应中断（Ctrl+C 时先停止小车再退出）

整个系统采用模块化管道架构，分为三大独立管线：
地图处理 → 路径规划 → 运动控制，每个管线都可独立运行，仅通过标准化数据文件交换信息。

2. 项目目录结构
text
robot_motion_project/
│
├── map_pipeline/                  # 地图处理管线（第一阶段）
│   ├── data/
│   │   ├── input/                 # 存放原始地图图片
│   │   └── output/
│   │       └── maps/              # 每张图处理后的专属文件夹
│   │           └── <地图名>/
│   │               ├── grid_map.npz       # 栅格地图数据
│   │               ├── start_goal.txt     # 起终点栅格坐标
│   │               ├── original.png       # 原图备份
│   │               ├── config.yaml        # 该地图的栅格化参数
│   │               └── grid_preview.png   # 栅格预览图（带网格线和起终点标记）
│   ├── src/
│   │   ├── grid_map.py            # GridMap 类定义
│   │   ├── map_loader.py          # 图片加载、二值化、栅格化
│   │   ├── interactive_picker.py  # 原图鼠标点击选点（像素坐标）
│   │   ├── coordinate_mapper.py   # 像素坐标 → 栅格坐标映射
│   │   ├── grid_resolution_tuner.py # 终端交互调整栅格尺寸
│   │   └── map_manager.py         # 多地图管理（预留）
│   ├── config.yaml                # 默认栅格化参数
│   └── run_pipeline.py            # 单图完整处理入口（选点→调参→栅格化→输出）
│
├── path_planner/                  # 全局路径规划管线（第二阶段）
│   ├── data/
│   │   ├── input/                 # 接收来自 map_pipeline 的标准化输入
│   │   │   ├── grid_map.npz
│   │   │   └── start_goal.txt
│   │   └── output/                # 路径规划结果
│   │       ├── path_data.npy      # 最终选定路径（N×2 栅格坐标数组）
│   │       ├── planned_path.png   # 最终路径可视化（单选时，含栅格网格）
│   │       ├── comparison.png     # 多算法对比图（多选时，含栅格网格）
│   │       └── final_path.png     # 确认后的最终路径图
│   ├── src/
│   │   ├── planner_base.py        # 抽象基类 GlobalPlanner + Path 类型定义
│   │   ├── planners/              # 所有路径规划算法集中存放
│   │   │   ├── __init__.py
│   │   │   ├── a_star.py          # A* 算法
│   │   │   ├── dijkstra.py        # Dijkstra 算法
│   │   │   ├── prm.py             # PRM 算法（概率路图）
│   │   │   ├── rrt.py             # RRT 算法（快速随机树）
│   │   │   └── rrt_star.py        # RRT* 算法（渐进最优）
│   │   ├── planner_factory.py     # 算法工厂（根据名称创建实例）
│   │   ├── algorithm_selector.py  # 终端交互：单选 / 多选算法
│   │   ├── comparator.py          # 多路径计算、耗时对比
│   │   ├── path_visualizer.py     # 路径可视化（单条、多条对比，绘制栅格线）
│   │   └── path_saver.py          # 保存最终路径为 .npy
│   ├── config.yaml                # 规划器配置（算法列表、交互模式等）
│   └── run_planner.py             # 规划入口：加载地图→选算法→计算→保存
│
├── motion_control/                # 运动控制与执行管线（第三阶段）
│   ├── data/
│   │   ├── input/
│   │   │   └── path_data.npy      # 来自 path_planner 的路径
│   │   └── output/
│   │       ├── trajectory.csv     # 行驶轨迹记录（可选）
│   │       └── control_log.csv    # 控制指令日志（可选）
│   ├── src/
│   │   ├── hardware_interface.py  # 硬件抽象基类
│   │   ├── ros2_interface.py      # 基于 ROS2 的麦克纳姆轮底盘控制（无侧滑）
│   │   ├── robot_model.py         # 仿真用运动学模型（预留）
│   │   └── trajectory_logger.py   # 日志记录工具
│   ├── config.yaml                # 控制参数（速度、栅格尺寸等）
│   └── run_follower.py            # 入口：加载路径 → 执行跟踪，Ctrl+C 安全停止
│
├── main.py                        # 统一调度入口（持久化循环，安全中断管理）
└── README.md                      # 本文档
3. 各模块详细功能说明
3.1 地图处理管线 (map_pipeline)
该管线负责将任意一张图片转换为可复用的标准化栅格地图，并确定起点和终点。

主要流程
图片加载：自动读取 data/input/ 中的第一张图片（兼容中文路径）。

原图选点：弹出窗口，用户在原始图片上点击起点（绿色）和终点（红色），按任意键确认，支持 R 重置。

交互调整栅格分辨率：终端显示当前栅格行列数，用户可输入目标宽度（列数）来调整精细度，回车确认。

二值化与栅格化：对图片进行灰度、高斯模糊、OTSU 自动阈值（或固定阈值）处理，将二值图降采样为栅格地图（0=空闲，1=障碍）。

坐标映射：将用户选中的像素坐标自动转换为栅格坐标，若落于障碍格则自动 BFS 搜索最近空闲格。

输出标准化文件：为每张地图生成专属目录 data/output/maps/<地图名>/，保存 grid_map.npz、start_goal.txt、原图备份、参数配置和栅格预览图（带网格线、起点绿色圆点、终点红色圆点）。

核心类与函数
文件	类 / 函数	功能
grid_map.py	GridMap	封装栅格矩阵，提供 is_free(row,col)、find_nearest_free()、save()/load() 等方法
map_loader.py	load_image()	兼容中文路径读取图片，返回 BGR 图像
binarize()	灰度化、高斯模糊、阈值处理生成二值图（支持 OTSU 或固定阈值）
create_grid_map()	根据目标宽度生成栅格地图，返回 GridMap 对象和缩放因子
interactive_picker.py	pick_points()	显示原图，鼠标点击选起点和终点，返回像素坐标，支持窗口缩放和重置
coordinate_mapper.py	map_pixel_to_grid()	根据 GridMap 的 cell_width/height 将像素坐标转换为栅格索引
grid_resolution_tuner.py	tune_resolution()	终端交互循环，允许用户输入新宽度来调整栅格分辨率，回车确认
run_pipeline.py	draw_grid_preview()	绘制带网格线和起终点标记的栅格预览图（灰度网格线）
safe_imwrite()	支持中文路径的图片保存，失败时自动改用二进制写入
main()	编排整个地图处理流程
输出文件说明
grid_map.npz：包含 grid（2D uint8 数组）、cell_width、cell_height、img_shape，可用 GridMap.load() 恢复。

start_goal.txt：两行，每行两个整数，分别为起点和终点的栅格坐标 row col。

original.png：原始图片的备份，用于后续重新选点。

config.yaml：该地图的栅格化参数（阈值、障碍占比、目标宽度等）。

grid_preview.png：栅格化后的黑白预览图，带有绿色起点、红色终点和灰色网格线。

3.2 路径规划管线 (path_planner)
该管线接收栅格地图和起终点，提供五种全局路径规划算法，并支持交互式选择和对比，最终输出带栅格网格的路径预览图。

主要流程
加载输入：读取 grid_map.npz 和 start_goal.txt。

算法选择：终端显示可用算法列表（A*、Dijkstra、PRM、RRT、RRT*），用户可单选或多选（支持 all）。

路径计算：对所选算法分别计算路径，记录步数和耗时。

结果预览：

单选：展示路径预览图 planned_path.png（含栅格网格），用户确认后保存。

多选：生成对比图 comparison.png（含栅格网格），并输出各算法的指标表格，用户选择其中一条作为最终路径，同时保存最终路径图 final_path.png。

输出最终路径：保存为 path_data.npy（N×2 栅格坐标数组）。

核心类与函数
文件	类 / 函数	功能
planner_base.py	GlobalPlanner(ABC)	规划器抽象基类，定义 plan(start, goal) -> Path 接口
planners/a_star.py	AStarPlanner	A* 算法（曼哈顿距离启发式）
planners/dijkstra.py	DijkstraPlanner	Dijkstra 算法（启发式为零）
planners/prm.py	PRMPlanner	概率路图算法（栅格采样 + Bresenham 碰撞检测 + Dijkstra 搜索）
planners/rrt.py	RRTPlanner	快速随机树算法（单树 RRT，Bresenham 碰撞检测）
planners/rrt_star.py	RRTStarPlanner	RRT* 算法（渐进最优，含重选父节点和重布线）
planner_factory.py	create_planner(name, grid)	根据算法名称创建对应的规划器实例，可扩展注册
algorithm_selector.py	select_algorithms(available)	终端交互，返回用户选择的算法名列表
comparator.py	compare_paths()	并行计算多条路径，返回各算法的路径、步数、耗时字典
path_visualizer.py	draw_single_path()	绘制单条路径，显示栅格地图（黑色障碍、白色空闲）、灰色网格线、青色路径、起点绿色圆点、终点红色叉号
draw_comparison()	在一张图上绘制多条路径进行对比，网格线可选
path_saver.py	save_final_path()	将路径数组保存为 .npy 文件
run_planner.py	main()	编排完整的规划交互流程，包含跨模块导入 GridMap
配置说明
config.yaml 主要参数：

interactive：是否开启交互模式（true 或 false）

default_algorithm：非交互模式默认使用的算法

available_algorithms：可供用户选择的算法列表

3.3 运动控制管线 (motion_control)
该管线加载规划好的路径，将栅格路径转换为电机控制指令，驱动树莓派麦克纳姆轮小车沿路径运动，到达终点后自动停止。已解决转向后侧滑问题：转向完成后只发送前进速度，不再发送横向速度。

主要流程
加载路径：读取 path_data.npy，计算相邻栅格间的移动方向（deltas）和转向角序列。

控制执行：

实车模式 (ros2)：基于 ROS2 发布 Twist 消息到 /cmd_vel 话题，采用“原地转向 → 直线前进一个格子”的状态机控制每个栅格段。

仿真模式：暂未实现，预留接口。

停止策略：到达终点或收到中断信号时，在 finally 块中发布一次全零 Twist 并取消定时器，确保小车停止。

中断处理：Ctrl+C 时通过信号传递，先让小车发送零速度再退出程序。

核心类与函数
文件	类 / 函数	功能
hardware_interface.py	HardwareInterface(ABC)	抽象基类，定义 execute_path() 和 stop()
ros2_interface.py	Ros2PathExecutor	实现硬件接口，封装 ROS2 节点，执行分段路径跟踪，finally 中强制停止
delta_to_dir(dr, dc)	将栅格位移转换为方向代号（0=N,1=E,2=S,3=W）
compute_turn_angles(directions)	根据方向序列计算每步需要的转向角度（度）
run_follower.py	main()	加载路径，生成 deltas 和 turn_angles，根据配置选择 ROS2 执行器并运行，添加信号处理
配置说明
config.yaml 主要参数：

mode：ros2 或 simulation

grid_size：每个栅格对应的实际移动距离（米）

turn_speed：转向角速度（rad/s）

move_speed：直行线速度（m/s）

timer_period：控制循环周期（秒）

方向映射（已修正）
转向完成后的移动阶段统一使用 linear.x = move_speed（前进），不再发送 linear.y 横向速度。小车通过原地转向调整方向，然后直接前进一个格子，避免了侧滑。

3.4 统一调度入口 (main.py)
main.py 是整个系统的顶层调度器，负责串联三个管线，提供持久的交互循环，并安全处理 Ctrl+C 中断。

功能
首次运行：若无已栅格化地图，自动启动地图处理。

地图选择：列出已有地图供用户选择，并将选中的地图文件复制到下游管线输入目录。

循环执行：依次执行路径规划 → 运动控制。

任务完成后：提示用户可重新选点、切换地图或退出。

中断管理：全局捕获 SIGINT，先终止运动控制子进程（子进程的 finally 会发送零速度），再退出主程序。

关键函数
run_script(script_path)：使用 subprocess.Popen 运行指定脚本，实时输出日志，返回是否成功；全局 current_process 记录当前子进程。

global_signal_handler(sig, frame)：收到中断信号时，尝试终止子进程并等待其清理，然后退出。

main()：主循环逻辑，包含地图管理、流程调度和用户交互菜单。

4. 数据流与模块间接口
模块间通过约定的数据文件进行解耦，传递关系如下：

text
[原始图片]
    ↓ (map_pipeline)
grid_map.npz + start_goal.txt
    ↓ (复制到 path_planner/data/input/)
path_data.npy
    ↓ (复制到 motion_control/data/input/)
[小车运动]
map_pipeline → path_planner：grid_map.npz、start_goal.txt

path_planner → motion_control：path_data.npy（栅格坐标序列）

所有中间文件均可持久化保存，支持地图复用和多次规划/控制。

5. 环境依赖与安装
5.1 基础依赖
所有模块需安装：

bash
pip install numpy opencv-python matplotlib pyyaml
5.2 实车控制（可选）
如果需要在树莓派小车上实际运行，还需要 ROS2（例如 Humble）以及 geometry_msgs。
确保 ROS2 环境已 source，并且 rclpy 可用。

6. 快速使用指南
6.1 准备地图图片
将地图图片放入 map_pipeline/data/input/ 中，支持 .jpg/.png/.bmp 等格式。
图片要求：浅色区域为可通行区域，深色为障碍物（可通过 config.yaml 调整阈值和反转）。

6.2 运行项目
在项目根目录执行：

bash
python main.py
6.3 交互步骤
首次运行：自动进行地图处理。在弹出的窗口点击起点（绿）和终点（红），按任意键确认；终端可调整栅格精细度，回车确认。

选择算法：终端列出 A*、Dijkstra、RRT、RRT*、PRM 等算法，输入序号（可多选，逗号分隔或 all）。

路径预览：

单选时查看预览图（含栅格网格），确认后保存路径。

多选时查看对比图和指标表格，选择最终使用的路径。

小车循迹：若已连接 ROS2 并配置正确，小车将自动沿规划路径行驶，到达终点后停止。

循环任务：完成后可选择“重新选点”、“切换地图”或“退出”。

中断程序：任何时候按 Ctrl+C，程序会先让小车停止，然后安全退出。

6.4 仅测试规划效果（无实车）
在 motion_control/config.yaml 中将 mode 设置为 simulation（目前仿真模式未实现），或直接单独运行前两个模块查看路径规划结果。

7. 常见问题
Q：找不到图片
确保图片放在 map_pipeline/data/input/ 中，且扩展名为程序支持的类型。

Q：起点/终点在障碍上
程序会自动寻找最近的空闲栅格，无需担心。

Q：找不到路径
可能是障碍物过多或起终点不连通。可尝试调整栅格分辨率或降低障碍阈值（修改 config.yaml）。

Q：实车运行方向错误
修改 ros2_interface.py 中 move 阶段的 linear.x 符号（正负）匹配小车实际前进方向。

Q：小车侧滑
最新版本已修复：转向完成后只发送前进速度，不再发送横向速度。若仍有问题，检查是否使用了旧版 ros2_interface.py。

Q：Ctrl+C 后小车仍在运动
最新版已在 main.py 和 ros2_interface.py 中增强了中断处理，确保子进程在退出前发送零速度。若问题依旧，请确认文件为最新版本。

Q：路径预览图没有栅格网格
最新版 path_visualizer.py 默认绘制网格线，检查是否使用了最新代码。

8. 扩展与自定义
新增路径规划算法：在 path_planner/src/planners/ 下添加新文件，实现 GlobalPlanner 接口，并在 planner_factory.py 中注册即可。

添加新控制器（如 PID、MPC）：在 motion_control/src/ 中创建新类，继承 HardwareInterface，然后在 run_follower.py 中根据配置实例化。

启用仿真模式：利用 robot_model.py 和 simulator.py（预留）搭建闭环仿真，方便算法调试。

重新选点功能：实现 map_pipeline/run_re_pick.py，调用 interactive_picker 和 coordinate_mapper 更新 start_goal.txt。