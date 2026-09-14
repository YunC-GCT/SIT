# map_pipeline — 地图处理

> 图片 → OTSU 二值化 → 栅格化 → 起终点映射。主输出为版本化 `map_result.json`。

---

## 快速开始

```bash
# 把图放入 data/input/，然后使用配置中的默认栅格宽度
python run_pipeline.py

# 只有主动调整时才进入栅格宽度设置
python run_pipeline.py --adjust-grid
```

1. 弹窗选起点(绿)→终点(红)，按 Enter 确认
2. `R` 重置，右键撤销最近一次点击，Esc 取消
3. 窗口显示映射后的栅格位置和空闲/障碍状态；障碍格不会自动挪点
4. 默认输出到 `runs/<run-id>/map_pipeline/`

---

## 两种调用方式

**交互式**：`python run_pipeline.py`（GUI 选点，默认使用配置栅格宽度）

**无显示器**：

```bash
python -m map_pipeline.json_pipeline data/input/map.png \
  --start 2,3 --goal 18,24 --output runs/map_result.json

python -m map_pipeline.json_pipeline data/input/map.png \
  --start-px 120,340 --goal-px 890,560 --output runs/map_result.json
```

无显示器调用必须成对提供起点和终点；未显式指定 `--preview` 时，预览保存为结果
JSON 同目录下的 `<结果文件名>_preview.png`。

**编程式**：

```python
from map_pipeline import MapPipeline
import yaml

cfg = yaml.safe_load(open("config.yaml"))
pipeline = MapPipeline(cfg)
gm, start, goal = pipeline.process("map.jpg", (120, 340), (890, 560))
pipeline.save_outputs(gm, start, goal, "runs/my-map/map_pipeline/")
```

---

## 核心模块

### `grid_map.py` — GridMap 类

```python
from map_pipeline.src.grid_map import GridMap

gm = GridMap(...)
print(gm.grid_size)       # 每格物理尺寸(m)
print(gm.is_free(10, 20)) # True=空闲
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
| `map_result.json` | 带版本、坐标系、物理尺度、起终点和预览路径的统一结果 |
| `<结果文件名>_preview.png` | 栅格预览（带网格线+起终点） |

---

## config.yaml

```yaml
default_width: 500      # 栅格宽度（列数）
threshold: 0            # 0=OTSU, 否则固定灰度值
obstacle_ratio: 0.5     # 黑色占比超此值→障碍
invert: false           # true=白色为障碍
blur_ksize: 5           # 高斯模糊核（奇数）
grid_size: 0.3          # 每格物理尺寸(m)，写入 map_result.json
```
