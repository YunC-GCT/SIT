# map_pipeline — 地图处理

> 图片 → 可选透视矫正 → 二值化与清理 → 栅格化 → 起终点映射。JSON CLI 默认输出 `runs/<run-id>/map_pipeline/result.json`。

---

## 快速开始

```bash
# 以下命令均在项目根目录执行；图片放入 map_pipeline/data/input/
python map_pipeline/run_pipeline.py

# 只有主动调整时才进入栅格宽度设置
python map_pipeline/run_pipeline.py --adjust-grid
```

1. 弹窗选起点(绿)→终点(红)，按 Enter 确认
2. `R` 重置，右键撤销最近一次点击，Esc 取消
3. 窗口显示映射后的栅格位置和空闲/障碍状态；障碍格不会自动挪点
4. 默认输出到 `runs/<run-id>/map_pipeline/`

---

## 两种调用方式

**交互式**：`python map_pipeline/run_pipeline.py`（优先 Tk 选点，按条件回退 OpenCV，默认使用配置栅格宽度）

**无显示器**：

```bash
python -m map_pipeline.json_pipeline map_pipeline/data/input/room-16room_000.png \
  --headless --start 313,424 --goal 311,427

python -m map_pipeline.json_pipeline map_pipeline/data/input/map.png \
  --headless --start-px 120,340 --goal-px 890,560 --output runs/map_result.json
```

非交互调用必须成对提供起点和终点；交互终端也可使用 `--select-terminal` 输入栅格坐标。未显式指定 `--preview` 时，预览保存为结果
JSON 同目录下的 `<结果文件名>_preview.png`。

**编程式**：

```python
from map_pipeline import MapPipeline
import yaml

with open("map_pipeline/config.yaml", encoding="utf-8") as handle:
    cfg = yaml.safe_load(handle)
pipeline = MapPipeline(cfg)
gm, start, goal = pipeline.process("map.jpg", (120, 340), (890, 560))
pipeline.save_outputs(gm, start, goal, "runs/my-map/map_pipeline/")
```

`MapPipeline` 提供基础图片到栅格 API，不执行 JSON CLI 的透视矫正和二值清理，也不自动将栅格列数限制为图片宽度。需要这些能力时使用 JSON CLI；基础 API 的 `default_width` 应不超过输入图片宽度。

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
from map_pipeline.src.map_loader import load_image, binarize, create_grid_map

img = load_image("中文路径.jpg")   # cv2.imdecode 兼容中文
binary = binarize(img)             # OTSU 自动阈值
binary = binarize(img, threshold=128, invert=True)  # 固定阈值+反转
gm, _ = create_grid_map(binary, target_width=500, obstacle_ratio=0.5, grid_size=0.3)
```

### `coordinate_mapper.py`

```python
from map_pipeline.src.coordinate_mapper import map_pixel_to_grid
row, col = map_pixel_to_grid(pixel_x, pixel_y, grid_map)
```

---

## 输出

| 文件 | 内容 |
|------|------|
| `result.json` | JSON CLI 的默认结果，含版本、坐标系、物理尺度、起终点和预览路径 |
| `map_result.json` | `MapPipeline.save_outputs()` 的基础地图结果，不自动生成预览 |
| `<结果文件名>_preview.png` | 栅格预览（带网格线+起终点） |

---

## config.yaml

```yaml
default_width: 512      # 与 Moving AI 原图列数一致
threshold: 127          # 黑白地图固定阈值
obstacle_ratio: 0.5     # 黑色占比超此值→障碍
invert: false           # true=白色为障碍
blur_ksize: 1           # 不模糊，保持细障碍
grid_size: 1.0          # 离线约定尺度，未标定
```

当前默认 512 列、每格 1.0 是 Moving AI 离线实验约定，不是场地实测尺度。JSON CLI 可在 `--rectify` 时使用 `--arena-width-m` 和 `--grid-size-m` 设置场地宽度与目标格长；示例和角点坐标语义见 [使用手册](../docs/程序介绍与使用手册.md#现场照片透视矫正)。

配置还支持 `rectify`、`rectify_output_width`、`arena_width_m`、`arena_height_m`、`min_obstacle_area_px` 和 `morphology_kernel`，默认关闭矫正及二值清理。小障碍清理可能同时移除真实小障碍，需检查输出预览。
