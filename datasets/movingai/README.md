# Moving AI 主实验地图

来源：[Moving AI 2D Grid Benchmarks](https://movingai.com/benchmarks/grids.html)。
下载日期、单文件官方地址、SHA-256、任务数及示例坐标见 `manifest.json`。

当前选择 12 张 512×512 原始地图：

- Room：`8room_000`、`16room_000`、`32room_000`、`64room_000`。
- Maze：`maze512-4-0`、`maze512-8-0`、`maze512-16-0`、`maze512-32-0`。
- Random：`random512-10-0`、`random512-20-0`、`random512-30-0`、`random512-40-0`。

本目录按类别保存官方 `.map` 和 `.scen` 原文件；主程序的 PNG 位于
`map_pipeline/data/input/`，文件名带 `room-`、`maze-`、`random-` 前缀。
由原始 `.map` 无损转换：一格一像素，`.`/`G` 为白色自由格，`@`/`O`/`T` 为黑色障碍。
当前地图配置为 512 列、阈值 127、模糊核 1、关闭清理和透视矫正，保持原始栅格。

在项目根目录执行 `python tools/import_movingai.py` 可重新下载这组地图，生成 PNG 和校验清单。
导入器仅依赖 Python 标准库；只写清单列出的地图，不清空其他输入。

`.scen` 的起终点是 x/y，CLI 使用 row/col，即 y/x；`manifest.json` 的
`example_task` 已转换。示例任务取各 `.scen` 的第一条，适合冒烟验证；
正式算法比较应覆盖不同长度的多条任务、重复随机种子并记录失败率。
官方参考长度基于八邻域模型，当前四邻域 A*/Dijkstra 和视线采样算法不能直接用该长度计算同口径最优率。

地图不含实车米制标定。默认 `grid_size=1.0` 只是离线尺度约定，指标优先使用栅格长度。
公开地图实验使用 `--no-drive` 或研究模式，实车另用真实场地和尺度配置。

数据归属与许可：Moving AI 官方声明采用
[Open Data Commons Attribution License 1.0](https://opendatacommons.org/licenses/by/1-0/)。
保留来源和归属，引用：Nathan Sturtevant, *Benchmarks for Grid-Based Pathfinding*,
IEEE Transactions on Computational Intelligence and AI in Games, 4(2), 144–148, 2012。
本组选用人工生成的 Room/Maze/Random，不包含商业游戏地图。

旧合成地图不再出现在主实验输入目录。`tests/fixtures/01_scattered.png` 仅用于
`tools/acceptance.py` 的快速软件验收，使用独立的 `tools/acceptance_map.yaml`；
`generate_test_maps.py` 也只生成到 `tests/fixtures/`。
