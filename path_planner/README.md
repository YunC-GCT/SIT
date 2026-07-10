# path_planner — 路径规划

> 栅格地图上运行五种全局路径规划算法，单选/多选对比，输出最优路径。

---

## 快速开始

```bash
# 确保 data/input/ 下有 grid_map.npz + start_goal.txt
python run_planner.py
```

选算法：序号 `1` / 逗号分隔 `1,3,5` / 全选 `all` → 计算 → 预览 → 确认保存 `path_data.npy`

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

### 多算法对比

```python
from src.comparator import compare_paths
results = compare_paths(['astar','rrt'], grid, start, goal, create_planner,
                         algo_params={'rrt': {'max_iter': 3000}})
for name, d in results.items():
    print(f"{name}: {d['steps']}步, {d['time']:.4f}s")
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
| **输入** | `grid_map.npz` + `start_goal.txt`（`validate_start_goal()` 校验：行数/整数/非负/边界） |
| **输出** | `path_data.npy` — N×2 int，每行 `[row, col]` |
