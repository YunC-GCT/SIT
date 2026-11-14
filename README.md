# SIT_LJGH_PROJECT — 机器人路径规划与循迹系统

> **当前状态：统一 JSON 流水线、交互恢复、路径后处理和 turn 控制已实现，曾生成 Fake ROS2 软件验收成功记录；实车方向、速度与路径跟随验收仍待完成。** 当前使用方式见 [整体程序介绍与使用手册](docs/程序介绍与使用手册.md)，设计约定见 [架构决策记录](docs/adr/) 和 [硬件与部署参考](docs/turbopi-hardware.md)。历史运行生成物已移出仓库，`runs/` 已加入 Git 忽略规则；历史验收记录不等于当前版本已重新验证。

> 大创项目：地图图片 → 栅格化 → 路径规划 → 路径后处理 → ROS2 TurboPi turn 模式循迹。各模块通过文件解耦，可独立运行。

---

## 阅读入口

- [程序介绍与使用手册](docs/程序介绍与使用手册.md)
- [项目设计 HTML 报告](docs/project-design-report.html)与[交互 HTML 报告](docs/interaction-report.html)：下载仓库后使用浏览器打开。
- [文档目录](docs/README.md)：使用、开发、硬件与实验记录。
- [接口清单](API_INDEX.md)与[领域词汇](CONTEXT.md)。

## 架构

```
地图图片 ──▶ map_pipeline ──▶ path_planner ──▶ path_postprocess ──▶ motion_control ──▶ /cmd_vel
             建图+选点       算法/变体选择      平滑/复核/分段        turn 模式循迹
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
pip install -r requirements.txt

# 普通模式：交互选点、路径预览和一次驱动前确认
python main.py

# 无显示器：显式提供地图和起终点，只生成预览，不驱动车
python main.py map_pipeline/data/input/room-16room_000.png \
  --headless --start 313,424 --goal 311,427 --no-drive

# 研究模式：多算法比较，默认不驱动车
python main.py map_pipeline/data/input/room-16room_000.png \
  --mode research --headless --start 313,424 --goal 311,427 \
  --algorithms astar,dijkstra --seed 42

# 或单独跑某条管线
python map_pipeline/run_pipeline.py
python -m path_planner.run_planner --map-result runs/<run-id>/map_pipeline/result.json
python -m path_postprocess.run_postprocess --map-result runs/<run-id>/map_pipeline/result.json --planner-result runs/<run-id>/path_planner/result.json
python motion_control/run_follower.py --postprocess-result runs/<run-id>/path_postprocess/result.json  # 需容器内 ROS2 环境
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
| **算法可插拔** | 实现含 `seed` 的规划器接口、导入注册并声明参数校验，即可复用统一流程 |
| **统一路径段** | motion_control 只读取 path_postprocess 的 `outputs.segments` |
| **坐标系** | 地图上方为北，默认初始车头为 `north`；ROS `angular.z` 按底盘坐标系发布 |
| **图形与无显示器输入** | 起终点选择优先 Tk，失败时按条件回退 OpenCV；路径预览用 OpenCV，无显示器可保存预览并显式传参 |

---

## 扩展

| 目标 | 操作 |
|------|------|
| **新增规划算法** | `planners/` 下实现 `GlobalPlanner` → `planner_factory.py` `_registry` 加一行 → `config.yaml` 加名字 |
| **新算法复用碰撞** | `from path_planner.src.grid_utils import line_of_sight` |
| **调速度/尺度** | 改对应 `config.yaml` |
| **初始朝向不对** | 改 `motion_control/config.yaml` 的 `initial_heading` |
| **测试运动控制** | 运行 `python -m unittest tests.test_motion_control_turn -v` |

---

## 环境

Python 3.10+ · ROS2 Humble（实车） · 依赖：`numpy opencv-python pyyaml` · 树莓派 Docker 预装 ROS2

---

## 当前实现边界

主实验输入已替换为 12 张 Moving AI Room/Maze/Random 地图，详见[地图清单与来源](datasets/movingai/README.md)。默认 512 列、固定阈值 127、模糊核 1，保持原始栅格；每格 1.0 仅为离线尺度约定，不代表真实米制标定。公开地图使用 `--no-drive` 或研究模式，真实场地另配尺度。

碰撞检测针对路径点和连线所在的障碍栅格，尚无车体轮廓或障碍膨胀配置。当前执行器始终使用计时开环；`odom_topic` 参数尚未连接反馈。路径预览中的预计时间来自后处理参数，当前后处理与执行器的默认转向速度不同，且执行还包含零速过渡，因此预计时间不是实测执行时长。

目标流程由可扩展的能力模块组成：地图处理 → 路径规划 → 路径后处理 → 运动控制。模块通过带版本和状态的 JSON 运行批次连接；未来可以加入定位、避障或里程计模块，而不修改已有模块的内部实现。

普通模式只要求地图、起点终点、路径预览和驱动前安全确认；研究模式用于多算法、参数、随机种子和指标比较。多算法结果需针对选定算法单独规划、后处理后再驱动。运动控制使用 `turn` 模式，发布到 `/cmd_vel`，当前始终使用经验速度和 timer 开环估算。

完整的模块边界、交互流程、错误恢复、测试接缝和容器运行说明见 [整体程序介绍与使用手册](docs/程序介绍与使用手册.md)。

公开地图来源、格式转换及实验口径见 [公开地图数据集选型](docs/public-map-datasets.md)。优先使用 Moving AI 的 Room、Maze、Random，自制小图仅保留在 tests/fixtures/ 作为快速软件回归夹具。

最新[实验结果总览](docs/verification/README.md)：软件回归 184 项通过；12 张图各 3 组任务的五算法全量测试成功 121/180，PRM 32/36、基础 RRT 10/36、RRT* 7/36；独立自适应 RRT 变体成功 6/36，未优于基础版。当前尚未全图跑通，实车待验收。早期[非真机流程验收](docs/verification/2026-11-14-offline.md)及其覆盖率仅为历史记录。运行 `python tools/offline_acceptance.py --output docs/verification/latest-offline.json` 可复验流程并自动清理临时日志。
