# SIT_LJGH_PROJECT — 机器人路径规划与循迹系统

> 大创项目:从一张地图图片出发,自动完成「建图 → 规划 → 循迹」全流程,驱动 ROS2 麦克纳姆轮小车从起点到达终点。

---

## 1. 项目简介

本系统把任意一张环境地图图片,自动栅格化为可复用的标准地图,然后在五种全局路径规划算法中任选一种(或多选对比)生成最优路径,最后通过 ROS2 把路径转成麦克纳姆轮小车的分段运动指令,完成自动循迹。

**核心能力**

- 一键将地图图片栅格化,支持中文路径
- 交互式鼠标选点 + 终端调栅格宽高(告知当前值,回车进入下一步)
- 五种全局路径规划算法:**A\*、Dijkstra、PRM、RRT、RRT\***
- 多算法对比可视化,带栅格网格线
- 基于 ROS2 的麦克纳姆轮实车控制,**原地转向 → 直线前进一格**的状态机控制
- **Ctrl+C 安全停止**:退出前必先把小车停稳,绝不"程序没了车还在跑"
- 顶层 `main.py` 串起三个管线,支持地图复用、循环任务

---

## 2. 系统架构

系统分三条独立管线 + 一个顶层调度器,管线之间**只通过标准化文件交换数据**,可以单独运行调试。

```
                  ┌─────────────────────────────────────┐
   原始图片  ──▶  │  map_pipeline  (地图处理)          │
                  │  图片 → 二值化 → 栅格化 → 起终点   │
                  └─────────────────────────────────────┘
                            │
                            ▼  grid_map.npz + start_goal.txt
                  ┌─────────────────────────────────────┐
                  │  path_planner  (路径规划)          │
                  │  加载栅格 → 选算法 → 计算 → 可视化 │
                  └─────────────────────────────────────┘
                            │
                            ▼  path_data.npy (N×2 栅格坐标)
                  ┌─────────────────────────────────────┐
                  │  motion_control  (运动控制)        │
                  │  分段解析 → ROS2 /cmd_vel → 小车   │
                  └─────────────────────────────────────┘
```

---

## 3. 目录结构

```
SIT_LJGH_PROJECT/
├── main.py                       # 顶层调度入口(串联三管线 + 循环 + 安全中断)
├── README.md                     # 本文档
├── .gitignore
│
├── map_pipeline/                 # ── 阶段1: 地图处理 ──
│   ├── config.yaml
│   ├── run_pipeline.py           # 单图完整处理入口
│   ├── run_batch.py              # 批量处理(预留)
│   ├── src/
│   │   ├── grid_map.py           # GridMap 类(BFS 找最近空闲格)
│   │   ├── map_loader.py         # 中文路径图片加载 + OTSU 二值化 + 栅格化
│   │   ├── interactive_picker.py # OpenCV 鼠标选点
│   │   ├── coordinate_mapper.py  # 像素 → 栅格坐标
│   │   └── grid_resolution_tuner.py  # 终端告知当前宽高,输入新值或回车
│   └── data/
│       ├── input/                # 原始地图图片
│       └── output/maps/<地图名>/ # 每张图处理后的专属文件夹
│           ├── grid_map.npz
│           ├── start_goal.txt
│           ├── original.png
│           ├── config.yaml
│           └── grid_preview.png
│
├── path_planner/                 # ── 阶段2: 路径规划 ──
│   ├── config.yaml
│   ├── run_planner.py            # 规划入口
│   ├── src/
│   │   ├── planner_base.py       # GlobalPlanner 抽象基类
│   │   ├── planner_factory.py    # 5 个算法的注册表
│   │   ├── algorithm_selector.py # 终端交互选算法(支持多选/all)
│   │   ├── comparator.py         # 多算法耗时对比
│   │   ├── path_visualizer.py    # matplotlib 可视化(带栅格线)
│   │   ├── path_saver.py         # 保存最终路径 .npy
│   │   └── planners/
│   │       ├── a_star.py         # A*(曼哈顿启发式)
│   │       ├── dijkstra.py       # Dijkstra(启发式=0)
│   │       ├── prm.py            # 概率路图 + Bresenham + Dijkstra
│   │       ├── rrt.py            # 快速随机树
│   │       └── rrt_star.py       # RRT*(渐进最优,重选父节点 + 重布线)
│   └── data/
│       ├── input/                # 来自 map_pipeline
│       └── output/               # path_data.npy / 对比图 / 最终路径图
│
└── motion_control/               # ── 阶段3: 运动控制 ──
    ├── config.yaml
    ├── run_follower.py           # 加载路径 → ROS2 执行
    └── src/
        ├── hardware_interface.py # 抽象基类
        ├── ros2_interface.py     # ROS2 节点 + 状态机 + 安全停车
        ├── robot_model.py        # MecanumModel(预留仿真)
        └── trajectory_logger.py  # CSV 日志工具
```

---

## 4. 各模块详细说明

### 4.1 地图处理 (map_pipeline)

把任意地图图片(`.jpg`/`.png`/`.bmp`)转换成标准栅格地图,并确定起终点。

**流程**

1. **图片加载** — 自动读取 `data/input/` 第一张图,兼容中文路径
2. **原图选点** — 弹出 OpenCV 窗口,鼠标依次点击起点(绿)和终点(红),按任意键确认,按 `R` 重置
3. **栅格宽高调整** — 终端先告知当前栅格行×列,用户输入新宽度(整数)或**直接回车**保持当前
4. **二值化与栅格化** — 灰度 → 高斯模糊 → OTSU 自动阈值 → 按目标宽度降采样为栅格(0=空闲,1=障碍)
5. **坐标映射** — 像素坐标 → 栅格索引;若起终点落在障碍格,**自动 BFS 找最近空闲格**
6. **输出** — 为每张地图生成专属目录 `data/output/maps/<地图名>/`,保存栅格、起终点、原图备份、参数配置、栅格预览图

**输出文件**

| 文件 | 说明 |
|---|---|
| `grid_map.npz` | 含 grid(2D uint8)、cell_width、cell_height、img_shape |
| `start_goal.txt` | 两行,每行 `row col`,分别是起终点栅格坐标 |
| `original.png` | 原图备份 |
| `config.yaml` | 该地图的栅格化参数 |
| `grid_preview.png` | 灰度预览图,带栅格线、绿起点、红终点;同时会**弹出预览窗口**,按任意键关闭 |

**关键函数**

| 文件 | 函数 | 功能 |
|---|---|---|
| `grid_map.py` | `GridMap` | 栅格数据封装,提供 `is_free` / `find_nearest_free` / `save` / `load` |
| `map_loader.py` | `load_image` / `binarize` / `create_grid_map` | 中文路径读图、OTSU 二值化、按目标宽度栅格化 |
| `interactive_picker.py` | `pick_points` | 显示原图、鼠标点击选起终点,自动按窗口尺寸缩放 |
| `coordinate_mapper.py` | `map_pixel_to_grid` | 像素坐标 → 栅格坐标,带边界裁剪 |
| `grid_resolution_tuner.py` | `tune_resolution` | 终端告知当前宽高,输入新值或回车保持 |
| `run_pipeline.py` | `draw_grid_preview` | 画预览图 + 弹窗显示(与选点同样的 cv2.imshow 方式) |

### 4.2 路径规划 (path_planner)

接收栅格地图和起终点,提供五种全局路径规划算法,支持单选/多选对比,输出带栅格网格的可视化。

**五种算法对照**

| 算法 | 类型 | 启发式 | 最优性 | 适用场景 |
|---|---|---|---|---|
| **A\*** | 增量式图搜索 | 曼哈顿距离 | 是 | 最常用,栅格场景首选 |
| **Dijkstra** | 增量式图搜索 | 无 | 是 | 无明确目标方向时的稳健选择 |
| **PRM** | 采样式 | — | 概率最优 | 高维空间或多连通区域 |
| **RRT** | 采样式 | — | 否(快速但非最优) | 快速验证、复杂障碍 |
| **RRT\*** | 采样式 | — | 渐进最优 | 在 RRT 基础上加重布线,迭代越多越优 |

**流程**

1. 加载 `grid_map.npz` + `start_goal.txt`
2. 终端列出可用算法,用户**单选(序号)** 或 **多选(逗号分隔)** 或 **all**
3. 对所选算法分别计算路径,记录步数和耗时
4. **单选**:展示 `planned_path.png`(含栅格网格)→ 用户确认后保存
5. **多选**:生成 `comparison.png`(多种颜色路径叠加)→ 输出指标表 → 用户挑一条 → 保存 `final_path.png`
6. 输出 `path_data.npy`(N×2 栅格坐标数组)

**关键函数**

| 文件 | 类 / 函数 | 功能 |
|---|---|---|
| `planner_base.py` | `GlobalPlanner` / `Path` | 规划器抽象基类 |
| `planners/a_star.py` | `AStarPlanner` | A\* + 曼哈顿距离 |
| `planners/dijkstra.py` | `DijkstraPlanner` | Dijkstra |
| `planners/prm.py` | `PRMPlanner` | 随机采样 + K 近邻 + Bresenham + Dijkstra |
| `planners/rrt.py` | `RRTPlanner` | 单树 RRT,Bresenham 碰撞检测 |
| `planners/rrt_star.py` | `RRTStarPlanner` | RRT\* + 重选父节点 + 重布线 |
| `planner_factory.py` | `create_planner` | 算法名称 → 实例 注册表 |
| `algorithm_selector.py` | `select_algorithms` | 终端交互选算法 |
| `comparator.py` | `compare_paths` | 多算法并行计算 + 计时 |
| `path_visualizer.py` | `draw_single_path` / `draw_comparison` | matplotlib 绘图(带栅格线) |
| `path_saver.py` | `save_final_path` | 保存为 .npy |

### 4.3 运动控制 (motion_control)

加载规划好的路径,转成麦克纳姆轮小车的运动指令,沿路径分段执行。

**核心:分段状态机**

每个相邻栅格对视为一段,执行顺序固定为「**原地转向 → 直线前进一个栅格**」,重复直到终点:

```
        ┌──────────────────────┐
        │  phase = 'turn'      │  angular.z = ±turn_speed
        │  累计角度 ≥ 目标 -1° │  progress += speed * dt
        └──────────┬───────────┘
                   ▼
        ┌──────────────────────┐
        │  phase = 'move'      │  linear.x = move_speed (不再发 linear.y)
        │  累计距离 ≥ 栅格-0.01│  progress += speed * dt
        └──────────┬───────────┘
                   ▼
              index += 1
              回到 'turn'(下一段)
```

**为什么转向后只发 `linear.x`?** 麦克纳姆轮允许任意方向平移,但若转向后同时给横向速度,会产生侧滑。所以转向完成后只发前进速度,横向速度置 0,小车直走一个格子。

**安全特性(本版本核心改动):Ctrl+C 必停车再退出**

```python
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    # 收到 Ctrl+C,准备停车
    ...
finally:
    _stop_robot()             # ← 取消 timer + 连续 10 次发零速度 Twist
    time.sleep(0.1)           # ← 给 DDS 一点时间把消息送出去
    node.destroy_node()
    rclpy.shutdown()
```

之前版本的问题是 `pub.publish(Twist())` 只发一次就立刻 destroy + shutdown,那一帧零速度消息还没走完底层 DDS 就被丢了,电机订阅端根本收不到。**新版改成连续 10 次发(每次间隔 20ms) + 100ms 缓冲**,确保至少一帧零速度被电机驱动收到。

**关键函数**

| 文件 | 类 / 函数 | 功能 |
|---|---|---|
| `hardware_interface.py` | `HardwareInterface` | 抽象基类 |
| `ros2_interface.py` | `Ros2PathExecutor` | ROS2 节点 + 状态机 + 安全停车 |
| `ros2_interface.py` | `delta_to_dir` | 栅格位移 → 方向代号(N/E/S/W) |
| `ros2_interface.py` | `compute_turn_angles` | 方向序列 → 转向角序列(度) |
| `robot_model.py` | `MecanumModel` | 麦克纳姆轮运动学模型(预留仿真) |
| `trajectory_logger.py` | `save_csv` | CSV 日志 |

**config.yaml 关键参数**

```yaml
motion_control:
  mode: "ros2"                  # "ros2" 或 "simulation"
  grid_size: 0.3                # 每栅格对应实际米数
  turn_speed: 2.0               # 转向角速度 (rad/s)
  move_speed: 0.3               # 直行线速度 (m/s)
  timer_period: 0.1             # 控制周期 (秒)
```

### 4.4 顶层调度 (main.py)

把三个管线串成一个持久循环:

```
[首次或切换地图] → run_pipeline.py
[循环开始]
    → run_planner.py
    → 把 path_data.npy 复制到 motion_control/data/input/
    → run_follower.py
    → 询问:r 重新选点 / m 切换地图 / q 退出
[循环结束]
```

**安全中断**:全局捕获 SIGINT,先终止当前子进程(子进程的 `finally` 会触发停车),再退出主程序。

---

## 5. 快速开始

### 5.1 安装依赖

```bash
pip install numpy opencv-python matplotlib pyyaml
```

实车控制额外需要 ROS2(例如 Humble)和 `geometry_msgs`,确保 ROS2 环境已 `source`。

### 5.2 准备地图

把地图图片放入 `map_pipeline/data/input/`,支持 `.jpg`/`.png`/`.bmp`。
约定:**浅色=可通行,深色=障碍物**(可通过 `config.yaml` 反转)。

### 5.3 启动

在项目根目录:

```bash
python main.py
```

### 5.4 交互步骤

1. **首次运行**(或切到新地图):自动开窗 → 鼠标点起点(绿)→ 点终点(红) → 按任意键
2. **栅格宽高**:终端告知当前栅格行×列,输入新宽度或**直接回车**保持
3. **栅格预览**:弹窗显示栅格地图,按任意键关闭
4. **选算法**:终端列出 5 种算法,输入序号(可多选逗号分隔或 `all`)
5. **路径确认**:
   - 单选 → 看预览图,输入 `y` 确认
   - 多选 → 看对比图,挑一条
6. **小车循迹**:小车自动沿路径行驶,完成自动停
7. **循环**:任务完成后可输入 `r` 重新选点 / `m` 切换地图 / `q` 退出
8. **中断**:任何时候 **Ctrl+C**,程序会**先让小车停车再退出**

### 5.5 单跑某个管线

每个管线都是独立入口,可以单独跑:

```bash
python map_pipeline/run_pipeline.py    # 只跑建图
python path_planner/run_planner.py     # 只跑规划(需先复制 grid_map.npz 到 data/input/)
python motion_control/run_follower.py  # 只跑控制(需先复制 path_data.npy 到 data/input/)
```

---

## 6. 数据流与模块间接口

模块之间只通过约定的文件解耦,**任何中间文件都可持久化保存**,支持地图复用和多次规划/控制。

```
map_pipeline  →  grid_map.npz + start_goal.txt
                →  path_planner/data/input/

path_planner  →  path_data.npy (N×2 栅格坐标数组)
                →  motion_control/data/input/

motion_control →  /cmd_vel (ROS2 Twist 消息)
                 →  小车电机驱动订阅
```

---

## 7. 常见问题

**Q: 找不到图片**
确保图片放在 `map_pipeline/data/input/` 下,扩展名受支持。

**Q: 起点/终点在障碍上**
程序自动 BFS 找最近空闲格,无需手动处理。

**Q: 找不到路径**
可能是障碍过多或起终点不连通。尝试降低栅格宽度(更精细)或修改 `obstacle_ratio` 阈值。

**Q: 实车方向反了**
修改 `ros2_interface.py` 中 `move` 阶段的 `linear.x` 符号(正负)。

**Q: 小车侧滑**
本版本已修复:转向完成后只发 `linear.x`,不再发 `linear.y`。若仍侧滑,检查是否使用最新版 `ros2_interface.py`。

**Q: Ctrl+C 后小车仍在跑**
本版本已修复:`_stop_robot()` 会连续发 10 次零速度 Twist + 100ms 缓冲。若仍有问题,确认 `ros2_interface.py` 是最新版。

---

## 8. 扩展指南

| 想做的事 | 改哪里 |
|---|---|
| 新增路径规划算法 | `path_planner/src/planners/` 加文件实现 `GlobalPlanner`,在 `planner_factory.py` 注册 |
| 新增控制器(PID/MPC) | `motion_control/src/` 加类继承 `HardwareInterface`,在 `run_follower.py` 按配置实例化 |
| 启用仿真模式 | 用 `robot_model.py` 搭建闭环仿真,在 `run_follower.py` 添加仿真分支 |
| 重新选点 | 实现 `map_pipeline/run_re_pick.py`,复用 `interactive_picker` + `coordinate_mapper` |

---

## 9. 环境

- Python 3.10+(测试用 3.13)
- ROS2 Humble(实车)
- 依赖:`numpy` / `opencv-python` / `matplotlib` / `pyyaml`