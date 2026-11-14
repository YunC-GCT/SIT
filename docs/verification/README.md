# 实验结果总览

本目录保留精简实验记录和机器可读结果，原始运行日志、缓存与预览不纳入版本控制。以下均为离线实验，不代表实车验收通过。

## 当前全地图结果

12 张 Moving AI 地图（Room/Maze/Random 各 4 张），每图随机抽取 3 个官方任务。任务抽样种子 20261114，规划种子 43/44/45。算法使用相同任务，单项墙钟上限 30 秒，三个子进程并行。成功要求起终点正确、双向碰撞检查及默认后处理通过。

| 算法 | 成功 | 预算内未找到路径 | 外部超时 | 记录 |
| --- | --- | --- | --- | --- |
| A* | 36 | 0 | 0 | 最新五算法全量批次 |
| Dijkstra | 36 | 0 | 0 | 同上 |
| PRM | 32 | 4 | 0 | 同上，包含自动重试 |
| 基础 RRT | 10 | 4 | 22 | 同上，固定步长单树 |
| RRT* | 7 | 29 | 0 | 同上，首解/优化分阶段预算 |
| RRT Adaptive | 6 | 3 | 27 | 独立变体批次，未混入原 180 项 |

五算法 180 项中成功 121、未找到 37、超时 22。自适应变体另外 36 项，结果未优于基础 RRT，保留为后续研究对照。新注册项不改变旧报告的算法数量。

详见 [最新五算法全量复测](2026-11-14-latest-config-full-test.md)、[自适应变体全量对照](2026-11-14-rrt-adaptive-full-test.md)、[基线分离说明](2026-11-14-rrt-baseline-separation.md)。对应 JSON 保存实际任务、参数和状态。

## 已完成与仍需改进

- 导入 12 张公开地图及官方场景、来源与校验信息；保留无损栅格处理。
- 修复 PRM 采样语义、连边与建图效率，RRT* 父节点/后代代价维护及最终目标比较，无向边双向碰撞判定。
- 基础 RRT 与自适应变体分开注册、配置和统计，基础版不接受自适应参数。
- PRM 原五算法基线成功 14/36，最新 32/36；剩余失败为 Maze4 三组及 Maze16 第二组。RRT/RRT* 仍有较多搜索失败或超时，尚未全图跑通。
- 最终软件回归 151 项通过（PYTHONUTF8=1），不代表随机地图全部成功。早期 136 项覆盖率约 86% 为历史测量，未对当前版本重新测覆盖率。
- 尚未完成独立任务/多随机种子统计、严格运行性能基准、实际 ROS2/ARM 容器与真车验收。

## 历史记录与复现工具

[原五算法基线](2026-11-14-all-maps-three-tasks.md) 成功 91/180；[修复记录](2026-11-14-sampling-fixes.md)、[分场景调参](2026-11-14-sampling-parameter-tuning.md)、[分阶段策略](2026-11-14-adaptive-strategies.md) 为演进证据，不与当前批次混合统计。此前固定三个短任务的 9 项成功不是全图结论。

- 全量：`python tools/validate_public_maps.py --planner-config path_planner/config.yaml --timeout 30 --output docs/verification/all-maps-latest-config.json`
- 变体：`python tools/validate_rrt_adaptive.py`
- 单任务：`tools/probe_sampling.py`；参数筛选：`tools/tune_sampling_parameters.py`。
- 历史修复前对照脚本移至 `tools/diagnostics/benchmark_sampling_fixes.py`，依赖仓库外旧代码归档，仅供追溯。

耗时受并发负载影响，RRT* 的时间预算也会影响确定性；NO_PATH 和超时均不证明地图无路。当前记录只有每张图三个任务，不能称任意起终点保证成功。
