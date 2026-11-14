# path_planner — 路径规划

> 栅格地图上运行五种全局路径规划算法，输出统一 JSON 原始路径和实验指标。

---

## 快速开始

```bash
# map_pipeline 已生成 map_result.json 后运行单个算法
python -m path_planner.run_planner \
  --map-result runs/map-001/map_pipeline/result.json \
  --algorithm astar \
  --run-root runs \
  --run-id planner-001

# 研究模式：一次运行多个算法并使用同一随机种子
python -m path_planner.run_planner \
  --map-result runs/map-001/map_pipeline/result.json \
  --research --seed 42
```

结果保存在 `runs/<run-id>/path_planner/result.json`，失败结果仍会保留
结构化错误和 `module.log`。

---

## 五种算法

| 算法 | 当前实现 | 最优性范围 |
|------|------|------|
| **A\*** | 四邻域、单位边代价、曼哈顿启发 | 当前离散图上的最短路径 |
| **Dijkstra** | 四邻域、单位边代价、无启发 | 当前离散图上的最短路径 |
| **PRM** | 自由格采样、近邻路图、Dijkstra | 生成路图上的最短路径；不保证全局最优 |
| **RRT** | 目标偏置采样、单树扩展 | 不保证最优 |
| **RRT\*** | RRT 加父节点重选和局部重布线 | 有限迭代结果不保证最优；理论渐进性质不能作为当前实现的验收结论 |

采样算法允许斜向视线连接，A* 和 Dijkstra 仅允许四邻域移动，直接比较长度时应说明可行边集合不同。碰撞检测基于栅格点和连线，目前没有车体轮廓或障碍膨胀建模。

PRM 的 `num_samples` 为不含起终点的不重复自由点数，最多采完可用自由格；`k_neighbors` 为每点主动选取的可通视邻居数量，隔墙候选不占此额度，无向边反向加入后实际度数可以更高。建图以空间分桶限定候选距离。采样语义修正后，同一 seed 的点集与旧版不同，旧版结果不可当作新版可复现结果。RRT/RRT* 跳过重复坐标；RRT* 重布线同步后代代价，并按最终代价选择目标连接点。修复效果见 [对照记录](../docs/verification/2026-11-14-sampling-fixes.md)。

---

## 新增算法两步

### 当前采样策略配置

仓库 `config.yaml` 现启用：PRM 首轮 20000 个自由点、15 个可见近邻、边长上限 15，失败后重采样至 30000 点；开阔地图可显式覆盖为 6000/15/40，关闭重试则设 `retry_num_samples: 0`。重试会新建路图并计入总耗时，不是免费扩容；不会读取地图文件名决定策略。

`rrt` 是基础对照基线：单树、均匀自由点采样加目标偏置、欧氏最近节点、固定步长、碰撞后拒绝扩展、找到路径后返回。不接受 `adaptive_step` 参数。它是坐标取整的栅格 RRT，保留重复节点剔除和双向碰撞检查，并非连续空间原始实现的逐字复刻。

`rrt_adaptive` 为独立优化变体：`step_size: 5` 为上限，按附近障碍距离缩小，受阻时逐次减半，最低尝试 1 格。API/CLI 使用独立名称，JSON algorithm 也独立标识；不得合并进基础 RRT 的成功率。RRT* 仍可显式选择 adaptive_step，默认 false。策略变体在 Room 有改善，在 Random 有回退，不保证普遍优于基线。

RRT* 启用 `search_time_s: 15`（首解前）、`optimize_time_s: 2`、`optimize_iterations: 2000`（首解后），另受总 `max_iter: 200000` 限制，任一预算先到即结束相应阶段。时间在循环边界检查，单次扩展可能使实际时间略超；时间预算运行受机器负载影响，不能只靠 seed 保证路径完全一致。将两项时间设 0、optimize_iterations 设 null 可恢复运行完整迭代预算。

JSON `outputs.search_diagnostics` 记录 PRM 各轮实际点数、参数和结果，或 RRT* 首解时间、迭代与停止原因；API 直接创建规划器时通过其 diagnostics 属性读取。工厂出厂默认仍保留固定策略，主入口按 YAML 加载新配置，独立 API 必须显式传参。

这些策略只在部分原任务复测，尚未重新通过全部 180 项验收。不能将旧固定参数记录当作新版默认配置验收。

```python
# ① planners/my_algo.py
from ..planner_base import GlobalPlanner, Path
class MyAlgo(GlobalPlanner):
    def __init__(self, grid, param=100, seed=None):
        super().__init__(grid, seed=seed)
        self.param = param
    def plan(self, start, goal) -> Path:
        ...; return path
```

```python
# ② planner_factory.py 的 _registry 加一行
'my_algo': (MyAlgo, {'param': 100}),
```

在工厂中导入 `MyAlgo`，再向 `config.yaml` 的 `available_algorithms` 添加 `my_algo`。新增参数还需在 `validate_algorithm_parameters()` 中加入相应类型与范围校验；随机算法应使用基类提供的 `self.rng`。统一 CLI、结果和预览可复用已有流程。

---

## 核心 API

### 工厂

```python
from path_planner.src.planner_factory import create_planner, list_algorithms
planner = create_planner('rrt', grid, params={'max_iter': 3000})
```

### 共享碰撞检测

```python
from path_planner.src.grid_utils import line_of_sight
# True = 无碰撞、未出界
line_of_sight(grid, rows, cols, (r1,c1), (r2,c2))
```

### 统一规划接口

```python
from path_planner.src.planner_pipeline import run_algorithm, run_planning

result = run_algorithm(
    grid, (0, 0), (10, 10), "rrt",
    parameters={"max_iter": 3000},
    seed=42,
)

comparison = run_planning(
    grid, (0, 0), (10, 10), ["astar", "rrt"],
    parameters={"rrt": {"max_iter": 3000}},
    seed=42,
    mode="research",
)
```

---

## config.yaml

```yaml
path_planner:
  interactive: true
  default_algorithm: "astar"
  available_algorithms: [astar, dijkstra, prm, rrt, rrt_star]
  algorithm_params:       # 可选，覆盖 factory 默认值
    prm:  {num_samples: 500, k_neighbors: 10, max_edge_len: 30}
    rrt:  {max_iter: 2000, step_size: 5, goal_sample_rate: 0.1}
    rrt_star: {max_iter: 2000, step_size: 5, goal_sample_rate: 0.1, search_radius: 10.0}
```

---

## 数据约定

| 方向 | 格式 |
|------|------|
| **输入** | `map_pipeline` 的 success JSON（含二维 grid、grid_size_m、start、goal） |
| **输出** | `result.json` — 原始路径、算法参数、随机种子和统一指标 |

### 指标

单算法结果的 `outputs.metrics.values` 包含：

- `found_path`：是否找到路径
- `planning_time_s`：规划耗时
- `raw_node_count`：原始路径节点数
- `raw_path_length_grid` / `raw_path_length_m`：栅格和物理路径长度
- `random_seed`：本次运行使用的随机种子

研究模式的 `outputs.comparisons` 为相同输入下各算法结果的数组；
某个算法未找到路径时，该项为 `failed` 并包含 `NO_PATH` 结构化错误，其他算法的结果仍保存在数组中。PRM/RRT/RRT* 的有限预算失败标为 `search_budget_exhausted`，不能据此判定地图不连通；A*/Dijkstra 的搜索失败标为 `unreachable`。当前实现只要有算法失败，整个规划结果即为 `failed`，下游后处理拒绝读取；检查保留结果后，可仅选择成功算法重新运行。

512×512 公开地图的采样参数诊断见 [实测记录](../docs/verification/2026-11-14-sampling-diagnosis.md)。RRT 的步长、PRM 的采样密度和窄通道覆盖比单纯放大 RRT* 重布线半径更关键；默认参数不保证所有可达任务规划成功。
