# SIT_LJGH_PROJECT — 机器人路径规划与循迹系统

> **当前状态：设计规格已确定，业务代码正在按新架构重构。** 新实现完成前，旧代码和旧文件契约不代表当前目标流程；请以 [整体程序介绍与使用手册](docs/程序介绍与使用手册.md)、[正式规格](.scratch/path-planning-control-redesign/spec.md) 和已接受 ADR 为准。

> 大创项目：地图图片 → 栅格化 → 路径规划 → 路径后处理 → ROS2 TurboPi turn 模式循迹。各模块通过文件解耦，可独立运行。

---

## 架构

```
地图图片 ──▶ map_pipeline ──▶ path_planner ──▶ path_postprocess ──▶ motion_control ──▶ /cmd_vel
             建图+选点       五种算法选择      平滑/复核/分段        turn 模式循迹
```

| 管线 | 输入 | 输出 | 详读 |
|------|------|------|------|
| **map_pipeline** | 图片 (.jpg/.png/.bmp) | 地图 JSON（含 grid_size） | [→ README](map_pipeline/README.md) |
| **path_planner** | 地图 JSON | 原始路径 JSON | [→ README](path_planner/README.md) |
| **path_postprocess** | 地图 JSON + 原始路径 JSON | 平滑路径和执行段 JSON | [→ README](path_postprocess/README.md) |
| **motion_control** | 已复核路径段 JSON | ROS2 `/cmd_vel`（Twist） | [→ README](motion_control/README.md) |

---

## 快速开始

```bash
pip install numpy opencv-python pyyaml

# 普通模式：交互选点、路径预览和一次驱动前确认
python main.py

# 无显示器：显式提供地图和起终点，只生成预览，不驱动车
python main.py map_pipeline/data/input/01_scattered.png \
  --headless --start 0,0 --goal 7,7 --no-drive

# 研究模式：多算法比较，默认不驱动车
python main.py map_pipeline/data/input/01_scattered.png \
  --mode research --headless --start 0,0 --goal 7,7 \
  --algorithms astar,dijkstra --seed 42

# 或单独跑某条管线
python map_pipeline/run_pipeline.py
python path_planner/run_planner.py
python motion_control/run_follower.py   # 需 ROS2 环境
```

### 交互流程

1. **建图**：使用配置中的默认栅格宽度，在窗口中选择起点和终点；无显示器时必须显式提供 `--start` 与 `--goal`
2. **规划**：普通模式使用默认算法；研究模式可指定多算法、参数和随机种子
3. **预览**：保存原始路径、平滑路径、执行段和预计时间；普通模式确认后才进入运动控制
4. **循迹**：预检 `/cmd_vel`，小车按“转向→零速→前进→停止”执行
5. **循环**：完成后可重跑、换算法、重新后处理、重新选点、换地图或退出；任何时候 Ctrl+C 先停车再退出

---

## 目录

```
SIT_LJGH_PROJECT/
├── main.py               # 普通/研究模式 CLI
├── core/
│   ├── orchestrator.py   # 顶层调度、恢复和预览确认
│   ├── batch.py          # 运行批次和重试尝试
│   └── contracts.py      # 统一 JSON 契约
├── pipeline_config.py    # 共享常量 + validate_start_goal()
│
├── map_pipeline/         # 建图：图片→栅格→起终点
│   ├── config.yaml
│   ├── map_pipeline.py   # MapPipeline 深类（编程调用）
│   ├── run_pipeline.py   # 交互式 CLI
│   └── src/              # GridMap / map_loader / picker / coordinate_mapper
│
├── path_planner/          # 规划：栅格→路径（5 算法）
│   ├── config.yaml
│   ├── run_planner.py
│   └── src/
│       ├── grid_utils.py      # line_of_sight() 共享碰撞检测
│       ├── planner_factory.py # 注册表 (class + defaults) + create_planner()
│       └── planners/          # astar / dijkstra / prm / rrt / rrt_star
│
├── path_postprocess/      # 后处理：平滑→碰撞复核→执行段
│   ├── config.yaml
│   ├── run_postprocess.py
│   └── src/
│       ├── smoothers.py
│       └── path_postprocessor.py
│
└── motion_control/        # 循迹：统一路径段→TurboPi /cmd_vel
    ├── config.yaml          # turn 模式、正北初始车头和安全停车参数
    ├── run_follower.py      # 读取 path_postprocess result.json
    └── src/ros2_interface.py  # turn 模式 + 开环估算 + 安全停车

docs/
└── robot_direction_reference.png  # 小车坐标系参考图
```

---

## 核心特性

| 特性 | 说明 |
|------|------|
| **Turn 运动控制** | 每段先原地转向，再零速过渡，沿车头方向前进并停止 |
| **Timer 开环估算** | 当前实车没有 `/odom`，使用可配置的约 `0.30 m/s` 经验速度 |
| **驱动前预检** | 检查 `/cmd_vel` 底盘订阅者和其他发布者，未确认时不驱动 |
| **安全停车** | Ctrl+C → 连发 10 次零速度 Twist + DDS 缓冲 → 再退出 |
| **算法可插拔** | 新算法 = 实现 `plan()` + 注册一行 → 自动接入全流程 |
| **统一路径段** | motion_control 只读取 path_postprocess 的 `outputs.segments` |
| **坐标系** | 地图上方为北，默认初始车头为 `north`；ROS `angular.z` 按底盘坐标系发布 |
| **cv2 统一预览** | 选点/栅格/路径/对比全部 cv2.imshow 弹窗，零 matplotlib 依赖 |

---

## 扩展

| 目标 | 操作 |
|------|------|
| **新增规划算法** | `planners/` 下实现 `GlobalPlanner` → `planner_factory.py` `_registry` 加一行 → `config.yaml` 加名字 |
| **新算法复用碰撞** | `from src.grid_utils import line_of_sight` |
| **调速度/尺度** | 改对应 `config.yaml` |
| **初始朝向不对** | 改 `motion_control/config.yaml` 的 `initial_heading` |
| **测试运动控制** | 运行 `python -m unittest tests.test_motion_control_turn -v` |

---

## 环境

Python 3.10+ · ROS2 Humble（实车） · 依赖：`numpy opencv-python pyyaml` · 树莓派 Docker 预装 ROS2

---

## 重构后的程序设计

目标流程由可扩展的能力模块组成：地图处理 → 路径规划 → 路径后处理 → 运动控制。模块通过带版本和状态的 JSON 运行批次连接；未来可以加入定位、避障或里程计模块，而不修改已有模块的内部实现。

普通模式只要求地图、起点终点、路径预览和驱动前安全确认；研究模式用于多算法、参数、随机种子和指标比较。运动控制默认采用 `turn` 模式，发布到 `/cmd_vel`，当前没有 `/odom` 时使用经验速度和 timer 开环估算。

完整的模块边界、交互流程、错误恢复、测试接缝和容器运行说明见 [整体程序介绍与使用手册](docs/程序介绍与使用手册.md)。
