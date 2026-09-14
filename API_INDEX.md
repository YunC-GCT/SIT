# 项目接口清单（后端视角）

> 以四个能力模块为索引，列出所有公开类、函数、常量。交互式 UI 函数标记为 `[UI]`。

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
| `core.RunBatch` | `class` | 创建独立运行批次目录，维护 `manifest.json`、模块状态和日志路径 |
| `core.make_result(module, run_id, status, ...)` | `-> dict` | 生成统一外层 JSON 结果，含版本、状态、输入、输出、参数、日志、错误和时间 |
| `core.make_error(code, ..., stage, reason, input_ref, suggestion)` | `-> dict` | 生成结构化错误，供模块失败和下游恢复提示使用 |
| `core.write_result(path, result)` | `-> str` | 校验并原子写入结果；失败结果不会覆盖已有成功结果 |
| `core.read_result(path)` | `-> dict` | 读取并校验统一 JSON 结果 |
| `core.require_success(path, module=None)` | `-> dict` | 下游读取入口：拒绝无版本、非成功或模块不匹配结果 |
| `core.grid_map_payload(...)` | `-> dict` | 地图 JSON 表达，记录 row/col 坐标、维度、类型、单位和栅格尺度 |
| `core.path_payload(points)` | `-> dict` | 原始路径/平滑路径 JSON 表达，记录点数组形状、类型、单位和坐标含义 |
| `core.path_segment_payload(...)` | `-> dict` | 控制器使用的路径段 JSON 表达，含方向角、转向角、距离、预计时间和运动类型 |
| `core.metric_payload(values, units)` | `-> dict` | 指标 JSON 表达，值和单位分离 |
| `core.module_entry(name, status, ...)` | `-> dict` | 运行批次或结果中的模块清单项 |
| `python -m core.contracts validate <result.json>` | CLI | 校验统一 JSON 结果，可加 `--require-success` 和 `--module` |
| `python tools/minimal_contract_module.py` | CLI | 最小示例模块，生成成功或 `--fail` 失败运行批次、日志和结果 |

---

## 管线一：map_pipeline（建图）

### 入口

| 接口 | 签名 | 说明 |
|------|------|------|
| **`run_pipeline.py`** | `main()` | `[UI]` 交互式全流程 |
| **`json_pipeline.py`** | `main()` | CLI；支持显式栅格/像素起终点和无显示器预览 |
| `json_pipeline.process(...)` | `-> dict` | 生成版本化地图 JSON，障碍起终点直接失败 |
| **`MapPipeline`** | `class` | **编程调用深接口** |

### MapPipeline 类

| 方法 | 签名 | 说明 |
|------|------|------|
| `__init__(config: dict)` | — | 从 config.yaml 字典取 default_width/threshold/grid_size 等 |
| `process(image_path, start_px, goal_px)` | `-> (GridMap, start_grid, goal_grid)` | 一站式：load→二值化→栅格化→坐标映射→校验起终点 |
| `save_outputs(grid_map, start, goal, dir, run_id)` | `-> result.json path` | 保存版本化地图 JSON |

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
| `validate_grid_point(grid_map, point, name)` | `-> (row, col)` | 校验坐标范围及空闲状态，不移动坐标 |
| `validate_grid_points(grid_map, start, goal)` | `-> (start, goal)` | 校验起点和终点必须位于空闲栅格 |
| `pick_points(img)` | `-> (px1, px2)` | `[UI]` cv2 鼠标选点 |
| `tune_resolution(default, w, h, preview)` | `-> int` | `[UI]` 终端调栅格宽度 |

---

## 管线二：path_planner（规划）

### 入口

| 接口 | 签名 | 说明 |
|------|------|------|
| **`run_planner.py`** | `main(argv)` | JSON CLI；读取 map_pipeline 结果并保存规划结果 |
| **`path_planner.src.planner_pipeline.run_algorithm`** | `run_algorithm(grid, start, goal, algorithm, parameters, seed, ...)` | 单算法统一结果 |
| **`path_planner.src.planner_pipeline.run_planning`** | `run_planning(grid, start, goal, algorithms, parameters, seed, ...)` | 研究模式多算法比较 |

### 规划器基类

| 接口 | 签名 | 说明 |
|------|------|------|
| `Path` | `= List[Tuple[int,int]]` | 类型别名 |
| `GlobalPlanner` | `ABC` | 抽象基类 |
| `GlobalPlanner.__init__(grid, seed=None)` | — | 局部随机源和栅格输入 |
| `GlobalPlanner.plan(start, goal)` | `-> Path` | 抽象方法；无解返回 `[]` |

### 工厂（新增算法改这里）

| 接口 | 签名 | 说明 |
|------|------|------|
| `create_planner(name, grid, params=None, seed=None)` | `-> GlobalPlanner` | params 覆盖默认参数并支持随机种子 |
| `validate_algorithm_parameters(name, params=None)` | `-> dict` | 校验算法参数 |
| `resolve_algorithm_parameters(name, params=None)` | `-> dict` | 返回含默认值的有效参数 |
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
| `compare_paths(names, grid, start, goal, factory, params)` | `-> {name: {path, steps, time}}` | 旧内部对比辅助函数；统一 JSON 结果使用 `run_planning` |
| `draw_path(grid, path, start, goal)` | `-> ndarray` | `[UI]` cv2 路径预览图 |
| `draw_comparison(grid, results, start, goal)` | `-> ndarray` | `[UI]` cv2 多算法对比图 |
| `show(title, img)` | — | `[UI]` cv2 弹窗封装 |
| `select_algorithms(available)` | `-> List[str]` | `[UI]` 终端选算法 |

---

## 管线三：path_postprocess（路径后处理）

| 接口 | 签名 | 说明 |
|------|------|------|
| **`path_postprocess.run_postprocess.main`** | `main(argv)` | CLI；读取地图和规划 JSON，写入独立后处理结果 |
| **`path_postprocess.process_path`** | `process_path(grid, raw_path, grid_size_m, parameters, ...)` | 平滑、采样、碰撞复核、最大段长分解和统一 JSON 输出 |
| `path_postprocess.PathSmoother` | `ABC` | 可替换平滑器适配边界 |
| `path_postprocess.NaturalCubicSplineSmoother` | `smooth(path, sample_spacing_grid)` | 自然边界分段三次样条和累计弧长采样实现 |
| `path_postprocess.resolve_postprocess_parameters` | `-> dict` | 校验并合并路径后处理配置 |

成功结果的 `outputs` 包含 `raw_path`、浮点采样 `smoothed_path`、`segments`、
`smoothing` 和 `metrics`；平滑失败时回退到已验证原始路径，并在
`outputs.smoothing.fallback_reason` 中记录原因。

---

## 管线四：motion_control（循迹）

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
  ├─── map_pipeline ─── result.json ──────────┐
  │    (grid + grid_size + start/goal)         │
  │                                           │
  ├─── path_planner ─── result.json ──────────┤
  │    (raw_path + metrics)                   │
  │                                           │
  ├─── path_postprocess ─── result.json ──────┤
  │    (smoothed_path + segments)             │
  │                                           │
  └─── motion_control ─── /cmd_vel ───▶ 小车
       (validated segments; consumer integration pending)
```
