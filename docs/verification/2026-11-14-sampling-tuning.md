# 代表任务参数调节

继 [首次诊断](2026-11-14-sampling-diagnosis.md) 后继续调参。地图、官方任务、起终点与 seed=42 均不变，不修改规划算法或默认配置。

| 地图 | 算法 | 参数覆盖 | 找到/碰撞检查 | 本机耗时 |
| --- | --- | --- | --- | --- |
| Room 8room_000 | PRM | num_samples=20000, k_neighbors=30 | 未找到；15097 个随机自由点，起点分量 5 点 | 111.51 秒 |
| Maze maze512-4-0 | PRM | num_samples=20000, k_neighbors=30 | 未找到；15295 个随机自由点，起点分量 291 点 | 114.94 秒 |
| Maze maze512-4-0 | RRT* | step_size=1, max_iter=200000 | 找到，19 个路径节点，逐段检查通过 | 72.15 秒 |
| Room 8room_000 | PRM | num_samples=20000, k_neighbors=150 | 找到，13 个路径节点，逐段检查通过 | 129.49 秒 |
| Maze maze512-4-0 | PRM | num_samples=20000, k_neighbors=150 | 未找到；起点分量仍为 291 点 | 145.03 秒 |

未覆盖参数保持原值：PRM max_edge_len=30；RRT* goal_sample_rate=0.1、search_radius=10。时间包含诊断采集，并受并行进程影响，不作为正式性能基准。RRT* 会运行完整迭代预算，此处不是首次解耗时。

Room 原图存在 1 格宽门口；增大边长不能代替穿门转弯需要的采样点。在相同 20000 次采样下，只增加 K 从 30 到 150 就成功，起点分量从 5 点增至 14173 点，说明隔墙近邻占额会影响有效连边。

Maze RRT* 保存路径后的复测耗时 80.89 秒，默认后处理及禁用平滑后处理均成功，见 [机器记录](sampling-tuning/maze-rrt-star.json)。Room/Random 的 RRT、RRT* 及 Random PRM 也已补测后处理，默认与禁用平滑均通过，记录位于同目录。

Room PRM 保存路径后的复测耗时 132.28 秒，默认与禁用平滑后处理均通过；Maze RRT 保存路径后的复测耗时 53.88 秒，两种后处理均通过。当前成功组合共 8 项，对应 JSON 均包含路径及后处理状态。规划模块 15 项 unittest 通过，探测工具 pyflakes 无警告。

用户要求停止继续搜索后，已终止 Maze PRM 的 50000 次采样、30 近邻进程；该项记为人工中止，无最终路径判定，不记作 NO_PATH。不再扩大采样量。算法检查见 [实现检查记录](2026-11-14-planner-implementation-check.md)。
