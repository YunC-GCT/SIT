# 项目接口清单（后端视角）

> 以当前四个能力分组为索引，列出主要入口与常用接口；不保证穷举所有导出符号。交互式 UI 函数标记为 `[UI]`。示例导入以项目根目录为基准，精确参数与关键字限制以源码签名为准。

---

## 集成入口

| 接口 | 签名 | 说明 |
|------|------|------|
| **`main.py`** | `main(argv=None)` | 普通/研究模式 CLI 总入口 |
| `core.run_pipeline(image_path, **kwargs)` | `-> PipelineReport` | 通过模块 CLI 契约调度完整流程 |
| `core.PipelineOrchestrator` | `class` | 运行批次、恢复、预览确认和运动门禁 |
| `core.save_path_preview(...)` | `-> str` | 从地图、规划和后处理 JSON 生成路径预览 |
| `run_script(path)` | `-> bool` | 兼容入口：启动单个模块脚本 |
| `core.map_selection.choose_map(...)` | `-> str \| None` | `[UI]` 启动、换地图和恢复共用编号/路径选择器；校验文件，错误重输，`b` 取消且不修改调用方状态 |
| `core.algorithm_selection.choose_algorithms(...)` | `-> list[str] \| None` | `[UI]` 从规划模块公开目录按编号或名称选算法；支持多选，错误重输，回车/`b` 保留当前选择 |
| `path_planner.catalog.algorithm_catalog(config_path=None)` | `-> (list[dict[str, str]], str)` | 按规划配置返回可选算法名称、显示说明和默认算法；注册表为算法名称的唯一来源 |

---

## 共享层

| 接口 | 签名 | 说明 |
|------|------|------|
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
| `python tools/offline_acceptance.py --output <report.json>` | CLI | 12 张公开地图、实际模块 CLI、Fake ROS2 和失败门禁验收，临时日志自动清理 |

`RunBatch` 会在同一 `run_id` 下续写已有 manifest；重试使用
`<module>/attempts/attempt-N/result.json`，失败尝试不会覆盖已成功阶段。

---

## 管线一：map_pipeline（建图）

### 入口

| 接口 | 签名 | 说明 |
|------|------|------|
| **`run_pipeline.py`** | `main()` | `[UI]` 交互式全流程 |
| **`json_pipeline.py`** | `main()` | CLI；支持显式栅格/像素起终点和无显示器预览 |
| `json_pipeline.process(...)` | `-> dict` | 生成版本化地图 JSON，障碍起终点直接失败 |
| `map_pipeline.process.prepare_grid_map(image, config)` | `-> GridMap` | 按配置二值化、清理、栅格化，将目标列数限制为图片宽度 |
| `map_pipeline.src.rectify.rectify_image(image, corners, ...)` | `-> (image, metadata)` | 四角透视矫正，返回变换元数据 |
| `map_pipeline.src.rectify.transform_points(points, homography)` | `-> list` | 将原图像素点变换到矫正图坐标，具体参数见源码 |
| **`MapPipeline`** | `class` | **编程调用深接口** |

### MapPipeline 类

| 方法 | 签名 | 说明 |
|------|------|------|
| `__init__(config: dict)` | — | 从 config.yaml 字典取 default_width/threshold/grid_size 等 |
| `process(image_path, start_px, goal_px)` | `-> (GridMap, start_grid, goal_grid)` | 一站式：load→二值化→栅格化→坐标映射→校验起终点 |
| `save_outputs(grid_map, start, goal, dir, run_id)` | `-> map_result.json path` | 保存基础版本化地图 JSON，不自动生成预览 |

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
| `clean_binary(binary, *, min_area_px=0, morphology_kernel=0)` | `-> ndarray` | 小障碍连通域与形态学清理 |
| `create_grid_map(binary, target_width, obstacle_ratio=0.5, grid_size=0.3)` | `-> (GridMap, scale)` | 按目标列数降采样，列数须不超过图宽 |
| `map_pixel_to_grid(x, y, grid_map)` | `-> (row, col)` | 像素→栅格，边界裁剪 |
| `validate_grid_point(grid_map, point, name)` | `-> (row, col)` | 校验坐标范围及空闲状态，不移动坐标 |
| `validate_grid_points(grid_map, start, goal)` | `-> (start, goal)` | 校验起点和终点必须位于空闲栅格 |
| `pick_points(img, ...)` | `-> (px1, px2)` | `[UI]` 优先 Tk，按条件回退 OpenCV；支持栅格反馈 |
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
| `AStarPlanner` | `(grid, seed=None)` | 四邻域图搜索·曼哈顿 |
| `DijkstraPlanner` | `(grid, seed=None)` | 四邻域图搜索·无启发 |
| `PRMPlanner` | `(grid, num_samples=500, k_neighbors=10, max_edge_len=30.0, seed=None)` | 采样·路图 |
| `RRTPlanner` | `(grid, max_iter=2000, step_size=5.0, goal_sample_rate=0.1, seed=None)` | 采样·单树 |
| `RRTStarPlanner` | `(grid, max_iter=2000, step_size=5.0, goal_sample_rate=0.1, search_radius=10.0, seed=None)` | 采样·父节点重选与重布线（继承 RRT） |

### 共享工具

| 接口 | 签名 | 说明 |
|------|------|------|
| `line_of_sight(grid, rows, cols, p1, p2)` | `-> bool` | Bresenham 碰撞检测；True=畅通 |
| `draw_path(grid, path, start, goal)` | `-> ndarray` | `[UI]` cv2 路径预览图 |
| `draw_comparison(grid, results, start, goal)` | `-> ndarray` | `[UI]` cv2 多算法对比图 |

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

多算法 CLI 输出使用 `outputs.comparisons[*].outputs`，每项包含上述后处理输出；运动控制 CLI 只接受顶层 `outputs.segments`，需要针对选定算法单独生成后处理结果。

---

## 管线四：motion_control（循迹）

### 入口

| 接口 | 签名 | 说明 |
|------|------|------|
| **`run_follower.py`** | `main(argv=None)` | 读取 `path_postprocess` 结果段→ROS2 `/cmd_vel` 预检→turn 模式执行 |

### 转向执行与兼容工具

| 接口 | 签名 | 说明 |
|------|------|------|
| `delta_to_dir(dr, dc)` | `-> int` | (Δrow,Δcol) → 方向代号 0/1/2/3 |
| `compute_turn_angles(dirs, heading)` | `-> [deg,...]` | 按初始车头计算离散方向的最短转角 |
| `compute_segment_turns(segments, initial_heading)` | `-> [deg,...]` | 从统一路径段的 `direction_deg` 计算 turn 模式转角 |
| `split_turn_angle_deg(angle)` | `-> [deg,...]` | 180 度掉头拆为两个 90 度转向 |

### Ros2PathExecutor 类

| 方法 | 签名 | 说明 |
|------|------|------|
| `__init__(turn_speed, move_speed, grid_size, timer_period, odom_topic, initial_heading, ...)` | — | 参数化构造，含绝对 `/cmd_vel`、经验前进速度和停车策略 |
| `setup_ros2()` | `-> (Node, Publisher)` | 创建 ROS2 基础设施 |
| `preflight_check(...)` | `-> CmdVelPreflight` | 检查 `/cmd_vel` 底盘订阅者和其他发布者 |
| `execute_segments(segments, node=None, pub=None, ...)` | `-> ExecutionReport` | 主执行循环：转向→零速→前进→停止，timer 开环估算，finally 安全停车 |
| `stop_robot()` | `-> None` | 重复发布零速度 Twist 并留出通信缓冲 |
| `shutdown_once()` | `-> None` | 幂等关闭 ROS2 context |
| `close()` | `-> None` | 再次发送零速度，销毁节点并幂等关闭 ROS2 |

当前执行器没有 odom 订阅，`odom_topic` 为保留参数。`ExecutionReport.stopped` 记录零速度发布流程是否完成，不代表已通过真实车轮停转观察。

### 常量

| 接口 | 值 | 说明 |
|------|-----|------|
| `CMD_VEL_TOPIC` | `"/cmd_vel"` | TurboPi 底盘速度话题 |
| `CHASSIS_NODE_NAME` | `"mecanum_chassis_node"` | 预检期望看到的底盘订阅者 |
| `DIR_NORTH / EAST / SOUTH / WEST` | 0/1/2/3 | 方向代号 |

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
       (validated segments; turn mode)
```
