# map_pipeline — 地图处理

> 图片 → OTSU 二值化 → 栅格化 → 起终点映射。输出 `grid_map.npz` + `start_goal.txt`。

---

## 快速开始

```bash
# 把图放入 data/input/，然后
python run_pipeline.py
```

1. 弹窗选起点(绿)→终点(红)，按任意键确认，`R` 重置
2. 终端调栅格宽度（回车=保持默认 500）
3. 预览栅格图 → 按任意键 → 输出到 `data/output/maps/<地图名>/`

---

## 两种调用方式

**交互式**：`python run_pipeline.py`（GUI 选点 + 终端调参）

**编程式**：

```python
from map_pipeline import MapPipeline
import yaml

cfg = yaml.safe_load(open("config.yaml"))
pipeline = MapPipeline(cfg)
gm, start, goal = pipeline.process("map.jpg", (120, 340), (890, 560))
pipeline.save_outputs(gm, start, goal, "data/output/maps/my_map/")
```

---

## 核心模块

### `grid_map.py` — GridMap 类

```python
from src.grid_map import GridMap

gm = GridMap.load("grid_map.npz")
print(gm.grid_size)       # 每格物理尺寸(m)，写入 npz 供下游校验
print(gm.is_free(10, 20)) # True=空闲
free = gm.find_nearest_free(100, 200)  # BFS 最近空闲格（deque, O(n)）
```

### `map_loader.py`

```python
from src.map_loader import load_image, binarize, create_grid_map

img = load_image("中文路径.jpg")   # cv2.imdecode 兼容中文
binary = binarize(img)             # OTSU 自动阈值
binary = binarize(img, threshold=128, invert=True)  # 固定阈值+反转
gm, _ = create_grid_map(binary, target_width=500, obstacle_ratio=0.5, grid_size=0.3)
```

### `coordinate_mapper.py`

```python
from src.coordinate_mapper import map_pixel_to_grid
row, col = map_pixel_to_grid(pixel_x, pixel_y, grid_map)
```

---

## 输出

| 文件 | 内容 |
|------|------|
| `grid_map.npz` | `grid`(2D uint8) + `cell_width/height` + `img_shape` + **`grid_size`** |
| `start_goal.txt` | 两行 `row col` |
| `original.png` | 原图备份 |
| `grid_preview.png` | 栅格预览（带网格线+起终点） |

---

## config.yaml

```yaml
default_width: 500      # 栅格宽度（列数）
threshold: 0            # 0=OTSU, 否则固定灰度值
obstacle_ratio: 0.5     # 黑色占比超此值→障碍
invert: false           # true=白色为障碍
blur_ksize: 5           # 高斯模糊核（奇数）
grid_size: 0.3          # 每格物理尺寸(m)，写入 npz
```
