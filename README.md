# SIT_LJGH_PROJECT — 机器人路径规划与循迹系统

> 大创项目：地图图片 → 栅格化 → 路径规划 → ROS2 麦轮小车全向循迹。三条管线通过文件解耦，可独立运行。

---

## 架构

```
地图图片 ──▶ map_pipeline ──▶ grid_map.npz ──▶ path_planner ──▶ path_data.npy ──▶ motion_control ──▶ /cmd_vel
             建图+选点       + start_goal.txt    五种算法选择      (N×2 栅格坐标)     麦轮全向平移
```

| 管线 | 输入 | 输出 | 详读 |
|------|------|------|------|
| **map_pipeline** | 图片 (.jpg/.png/.bmp) | `grid_map.npz`（含 grid_size）+ `start_goal.txt` | [→ README](map_pipeline/README.md) |
| **path_planner** | grid_map.npz + start_goal.txt | `path_data.npy`（N×2 int） | [→ README](path_planner/README.md) |
| **motion_control** | path_data.npy + grid_map.npz | ROS2 `/cmd_vel`（Twist） | [→ README](motion_control/README.md) |

---

## 快速开始

```bash
pip install numpy opencv-python pyyaml

# 全流程一键启动
python main.py

# 或单独跑某条管线
python map_pipeline/run_pipeline.py
python path_planner/run_planner.py
python motion_control/run_follower.py   # 需 ROS2 环境
```

### 交互流程

1. **建图**：弹窗鼠标选起点(绿)→终点(红) → 终端调栅格宽度(回车=保持) → 自动 OTSU 二值化 → BFS 修正障碍点
2. **规划**：选算法（序号 / 逗号分隔多选 / `all`）→ 预览路径 → 确认保存
3. **循迹**：启动校验 grid_size → 小车按 odom 位移逐段平移（全程 `angular.z=0`，无转向）
4. **循环**：完成后 `m` 切换地图 / `q` 退出 / 回车重跑；任何时候 Ctrl+C 先停车再退出

---

## 目录

```
SIT_LJGH_PROJECT/
├── main.py               # 顶层调度（串联管线 + SIGINT 转发）
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
└── motion_control/        # 循迹：路径→麦轮全向平移
    ├── config.yaml          # initial_heading=east（小车物理朝向）
    ├── run_follower.py      # 加载路径 + interpolate_path 展开大步长
    └── src/ros2_interface.py  # 麦轮全向 + odom 闭环 + 安全停车

docs/
└── robot_direction_reference.png  # 小车坐标系参考图
```

---

## 核心特性

| 特性 | 说明 |
|------|------|
| **麦轮全向平移** | 不转向，直接用 `(vx,vy)` 走四个方向，`angular.z=0` |
| **odom 闭环** | 段切换由里程计位移驱动，odom 不可用时 timer 兜底 |
| **grid_size 校验** | 地图创建时写入物理尺度，循迹启动时自动校验一致性 |
| **安全停车** | Ctrl+C → 连发 10 次零速度 Twist + DDS 缓冲 → 再退出 |
| **算法可插拔** | 新算法 = 实现 `plan()` + 注册一行 → 自动接入全流程 |
| **路径插值** | `interpolate_path()` 自动将 PRM/RRT/RRT* 大步长展开为逐格步长，方向/距离均正确 |
| **坐标系** | 小车默认朝 **EAST**，速度指令以此为基准（全程 `angular.z=0` 不旋转），见 `docs/` |
| **cv2 统一预览** | 选点/栅格/路径/对比全部 cv2.imshow 弹窗，零 matplotlib 依赖 |

---

## 扩展

| 目标 | 操作 |
|------|------|
| **新增规划算法** | `planners/` 下实现 `GlobalPlanner` → `planner_factory.py` `_registry` 加一行 → `config.yaml` 加名字 |
| **新算法复用碰撞** | `from src.grid_utils import line_of_sight` |
| **调速度/尺度** | 改对应 `config.yaml` |
| **初始朝向不对** | 改 `motion_control/config.yaml` 的 `initial_heading`，默认 `east` |
| **仿真模式** | 注入 FakePublisher 到 `execute_path(node, pub)` |

---

## 环境

Python 3.10+ · ROS2 Humble（实车） · 依赖：`numpy opencv-python pyyaml` · 树莓派 Docker 预装 ROS2
