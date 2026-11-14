# 公开地图采样规划失败诊断

本记录补充同日离线验收，原始 9 次失败记录保持不变。使用原始 512×512 栅格、原起终点、seed=42，直接运行真实规划器，核对起终点及每段 Bresenham 碰撞检查；没有缩图、移动端点或调用 A* 兜底。直接 API 成功尚不等于后处理/实车验收通过。

## 可复现入口

```powershell
.\.venv\Scripts\python.exe tools/probe_sampling.py --require-path
.\.venv\Scripts\python.exe tools/probe_sampling.py --algorithm astar --require-path
.\.venv\Scripts\python.exe tools/probe_sampling.py --params '{"num_samples":3000}' --require-path
.\.venv\Scripts\python.exe tools/probe_sampling.py --algorithm rrt --params '{"step_size":1}' --require-path
```

第一条在默认 Random 任务返回退出码 1，后三条成功。工具只输出单行汇总，不生成批次或日志。

## 控制变量实测

未列参数沿用原默认值：PRM 500/10/30；RRT 2000/5/0.1；RRT* 额外半径 10。时间来自本机单次运行，部分进程并行，仅作定位参考。

| 图 | 算法 | 参数覆盖 | 结果 |
| --- | --- | --- | --- |
| Random | A* | 无 | 成功，93 个节点，约 0.005 秒 |
| Random | PRM | 无 | 失败，448 个随机自由点，起点连通分量仅 3 点 |
| Random | PRM | max_edge_len=100 | 失败，起点分量 4 点 |
| Random | PRM | k_neighbors=40 | 失败，起点分量 3 点 |
| Random | PRM | num_samples=3000 | 成功，11 个路径节点，约 2.74 秒 |
| Random | RRT | 无 | 失败 |
| Random | RRT | max_iter=20000 | 失败，约 23.4 秒 |
| Random | RRT | step_size=1 | 成功，82 个路径节点，约 0.01 秒 |
| Random | RRT* | search_radius=50 | 失败 |
| Random | RRT* | step_size=1 | 成功，13 个路径节点，约 0.67 秒 |
| Room | RRT | step_size=1 | 成功，77 个路径节点 |
| Room | RRT* | step_size=1 | 成功，19 个路径节点 |
| Room | PRM | num_samples=3000 | 失败，起终点均无连边 |
| Room | PRM | num_samples=8000, k_neighbors=30 | 失败，起点分量仅 3 点 |
| Maze | PRM | num_samples=3000 | 失败，起点分量 6 点 |
| Maze | PRM | num_samples=8000, k_neighbors=30 | 失败，起点分量 44 点 |
| Maze | RRT/RRT* | step_size=1, max_iter=20000 | 均失败 |
| Maze | RRT | step_size=2 或 3, max_iter=20000 | 均失败 |
| Maze | RRT | step_size=1, max_iter=200000 | 成功，86 个路径节点，约 45.17 秒 |

所有成功条目均通过逐段碰撞检查。Maze 的 RRT 在步长 1、20000 次迭代后有 293 个树节点、283 个不同坐标，距离目标最近仍约 27.2 格。最后两组组合参数只作补充探测，不用于归因某个单独参数。

## 原因与边界

1. A* 穷举可达栅格，四邻域成功证明这些任务有路；采样算法有限预算失败不能证明无路。
2. PRM 在整个 262144 格地图上只做 500 次采样尝试，障碍和重复点还会降低实际数量。小房间、窄门、迷宫拐角缺少必要节点，路图断开。实现先截取 K 个几何近邻再检查碰撞，隔墙邻居占用候选额度。增大边长无法穿墙，也不能替代缺失的转弯节点。
3. RRT 默认步长 5，要求整段无碰撞；窄通道/随机障碍会频繁拒绝扩展。全图均匀采样与几何最近节点选择在弯曲迷宫中还会反复尝试穿墙方向。步长缩小明显改善 Room/Random，但不是 Maze 的成功保证。
4. RRT 根本没有 search_radius。RRT* 的 search_radius 用于选择父节点与重布线，不是保证探索覆盖的搜索范围；实测放大至 50 没有解决 Random 失败。

此次只修正错误语义：采样算法空路径仍返回 NO_PATH，但 error_type 改为 search_budget_exhausted，明确不代表不连通；A*/Dijkstra 保留 unreachable。新增连通地图、单次 RRT 预算耗尽的回归测试，先观察失败再修复。

后续若要求这些代表任务稳定成功，需要单独评估 PRM 自由点采样语义、窄通道采样及有效近邻策略，以及 RRT 的局部探索/步长策略和运行预算；至少在多个任务、多个种子上检验成功率、碰撞和耗时。当前不能宣称九项规划均已成功，也不能给出适用于所有公开地图的固定半径。

## 修改验证

新增预算耗尽回归测试通过；启用 `PYTHONUTF8=1` 后完整 unittest 为 137/137 通过，修改文件 pyflakes 无警告。首次未启用 UTF-8 时，一项既有中文 CLI 测试返回 1 而非 2；启用 UTF-8 后通过，不能把该首次结果写成全绿。算法默认值与采样实现本轮未变更。
