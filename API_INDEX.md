# 项目接口清单（后端视角）

> 以三条管线为索引，列出所有公开类、函数、常量。交互式 UI 函数标记为 `[UI]`。

---

## 集成入口

| 接口 | 签名 | 说明 |
|------|------|------|
| **`main.py`** | `main()` | 全流程总入口 |
| `run_script(path)` | `-> bool` | 启动子进程（新进程组），转发 SIGINT |
| `select_and_load_map(...)` | `-> bool` | 选择或创建栅格化地图 |

---

## 共享层

| 接口 | 签名 | 说明 |
|------|------|------|
| `pipeline_config.GRID_MAP_FILENAME` | `= "grid_map.npz"` | 跨管线文件名常量 |
| `pipeline_config.START_GOAL_FILENAME` | `= "start_goal.txt"` | |
| `pipeline_config.PATH_DATA_FILENAME` | `= "path_data.npy"` | |
| `validate_start_goal(filepath, max_rows, max_cols)` | `-> (start, goal)` | 校验起终点文件，错误时友好报错 |

---

## 管线一：map_pipeline（建图）

### 入口

| 接口 | 签名 | 说明 |
|------|------|------|
| **`run_pipeline.py`** | `main()` | `[UI]` 交互式全流程 |
| **`MapPipeline`** | `class` | **编程调用深接口** |

### MapPipeline 类

| 方法 | 签名 | 说明 |
|------|------|------|
| `__init__(config: dict)` | — | 从 config.yaml 字典取 default_width/threshold/grid_size 等 |
| `process(image_path, start_px, goal_px)` | `-> (GridMap, start_grid, goal_grid)` | 一站式：load→二值化→栅格化→坐标映射→BFS 修正障碍点 |
| `save_outputs(grid_map, start, goal, dir)` | — | 保存 grid_map.npz + start_goal.txt |

### GridMap 类

| 方法 | 签名 | 说明 |
|------|------|------|
| `__init__(grid, cell_w, cell_h, img_shape, grid_size)` | — | grid: 0=空闲 1=障碍；grid_size: m/格 |
| `is_free(row, col)` | `-> bool` | 边界检查 + 空闲判定 |
| `find_nearest_free(row, col)` | `-> (row, col)` | BFS 最近空闲格；全障碍时抛 RuntimeError |
| `save(filepath)` | — | 存为 npz（含 grid_size） |
| `load(filepath)` | `-> GridMap` | 静态方法 |

### 底层函数

| 接口 | 签名 | 说明 |
|------|------|------|
| `load_image(path)` | `-> ndarray` | 中文路径兼容 |
| `binarize(img, threshold, blur_ksize, invert)` | `-> ndarray` | threshold=0→OTSU 自动 |
| `create_grid_map(binary, width, ratio, grid_size)` | `-> (GridMap, scale)` | 按目标列数降采样 |
| `map_pixel_to_grid(x, y, grid_map)` | `-> (row, col)` | 像素→栅格，边界裁剪 |
| `pick_points(img)` | `-> (px1, px2)` | `[UI]` cv2 鼠标选点 |
| `tune_resolution(default, w, h, preview)` | `-> int` | `[UI]` 终端调栅格宽度 |

---

## 管线二：path_planner（规划）

### 入口

| 接口 | 签名 | 说明 |
|------|------|------|
| **`run_planner.py`** | `main()` | `[UI]` 选算法→计算→可视化→保存 |

### 规划器基类

| 接口 | 签名 | 说明 |
|------|------|------|
| `Path` | `= List[Tuple[int,int]]` | 类型别名 |
| `GlobalPlanner` | `ABC` | 抽象基类 |
| `GlobalPlanner.__init__(grid)` | — | |
| `GlobalPlanner.plan(start, goal)` | `-> Path` | 抽象方法；无解返回 `[]` |

### 工厂（新增算法改这里）

| 接口 | 签名 | 说明 |
|------|------|------|
| `create_planner(name, grid, params=None)` | `-> GlobalPlanner` | params 覆盖默认参数 |
| `list_algorithms()` | `-> List[str]` | 所有已注册算法名 |
| `get_algorithm_info(name)` | `-> (类名, 默认参数字典)` | |

### 算法实现

| 类 | 构造函数参数 | 类型 |
|------|------|------|
| `AStarPlanner` | `(grid)` | 图搜索·曼哈顿 |
| `DijkstraPlanner` | `(grid)` | 图搜索·无启发 |
| `PRMPlanner` | `(grid, num_samples=500, k_neighbors=10, max_edge_len=30.0)` | 采样·路图 |
| `RRTPlanner` | `(grid, max_iter=2000, step_size=5.0, goal_sample_rate=0.1)` | 采样·单树 |
| `RRTStarPlanner` | `(grid, max_iter=2000, step_size=5.0, goal_sample_rate=0.1, search_radius=10.0)` | 采样·渐进最优（继承 RRT） |

### 共享工具

| 接口 | 签名 | 说明 |
|------|------|------|
| `line_of_sight(grid, rows, cols, p1, p2)` | `-> bool` | Bresenham 碰撞检测；True=畅通 |
| `compare_paths(names, grid, start, goal, factory, params)` | `-> {name: {path, steps, time}}` | 多算法计时对比 |
| `save_final_path(path, path)` | — | 存 .npy |
| `draw_path(grid, path, start, goal)` | `-> ndarray` | `[UI]` cv2 路径预览图 |
| `draw_comparison(grid, results, start, goal)` | `-> ndarray` | `[UI]` cv2 多算法对比图 |
| `show(title, img)` | — | `[UI]` cv2 弹窗封装 |
| `select_algorithms(available)` | `-> List[str]` | `[UI]` 终端选算法 |

---

## 管线三：motion_control（循迹）

### 入口

| 接口 | 签名 | 说明 |
|------|------|------|
| **`run_follower.py`** | `main()` | 加载路径→校验 grid_size→ROS2 执行 |

### 速度指令生成

| 接口 | 签名 | 说明 |
|------|------|------|
| `delta_to_dir(dr, dc)` | `-> int` | (Δrow,Δcol) → 方向代号 0/1/2/3 |
| `compute_vel_commands(dirs, heading, speed)` | `-> [(vx,vy),...]` | **麦轮全向**：方向→(vx,vy)，无旋转 |
| `compute_turn_angles(dirs, heading)` | `-> [deg,...]` | 旧版：方向→转向角序列（保留兼容） |

### Ros2PathExecutor 类

| 方法 | 签名 | 说明 |
|------|------|------|
| `__init__(turn_speed, move_speed, grid_size, timer_period, odom_topic, initial_heading)` | — | 参数化构造 |
| `setup_ros2()` | `-> (Node, Publisher)` | 创建 ROS2 基础设施 |
| `execute_path(deltas, vel_commands, node=None, pub=None)` | — | **主执行循环**：odom 闭环段切换+timer 兜底；finally 安全停车 |

### 常量

| 接口 | 值 | 说明 |
|------|-----|------|
| `DIR_NORTH / EAST / SOUTH / WEST` | 0/1/2/3 | 方向代号 |
| `SEGMENT_TIMEOUT_RATIO` | 1.5 | 段切换超时倍数 |
| `ODOM_MISSING_WARN_AFTER` | 30 | 无 odom 报警阈值 |

---

## 数据流

```
原始图片
  │
  ├─── map_pipeline ─── grid_map.npz ────────┐
  │    (grid + grid_size)                     │
  │    start_goal.txt ────────────────────────┤
  │                                           │
  ├─── path_planner ─── path_data.npy ────────┤
  │    (N×2 int)                              │
  │                                           │
  └─── motion_control ─── /cmd_vel ───▶ 小车
       (grid_map.npz 用于校验 grid_size)
```
