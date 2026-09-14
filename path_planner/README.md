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

| 算法 | 类型 | 最优 | 场景 |
|------|------|:----:|------|
| **A\*** | 图搜索·曼哈顿 | ✅ | 栅格首选 |
| **Dijkstra** | 图搜索·无启发 | ✅ | 无方向偏好 |
| **PRM** | 采样·路图+Dijkstra | 概率 | 多连通区域 |
| **RRT** | 采样·单树扩展 | ❌ | 快速验证 |
| **RRT\*** | 采样·RRT+重布线 | 渐进 | 路径质量优先 |

---

## 新增算法两步

```python
# ① planners/my_algo.py
from ..planner_base import GlobalPlanner, Path
class MyAlgo(GlobalPlanner):
    def __init__(self, grid, param=100):
        super().__init__(grid); self.param = param
    def plan(self, start, goal) -> Path:
        ...; return path
```

```python
# ② planner_factory.py 的 _registry 加一行
'my_algo': (MyAlgo, {'param': 100}),
```

然后 `config.yaml` 的 `available_algorithms` 加 `my_algo`。终端选算法、对比、可视化全自动适配。

---

## 核心 API

### 工厂

```python
from src.planner_factory import create_planner, list_algorithms
planner = create_planner('rrt', grid, params={'max_iter': 3000})
```

### 共享碰撞检测

```python
from src.grid_utils import line_of_sight
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
某个算法无解时，该项为 `failed` 并包含 `NO_PATH` 结构化错误，其他算法仍可比较。
