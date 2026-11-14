# 公开地图数据集选型

访问与核对日期：2026-11-14。本文针对当前“地图图片 → 栅格 → 全局规划 → 后处理”的流程筛选数据，不把公开地图等同于实车场地或实测尺度。本次核对官方页面、格式说明和作者仓库，随后已导入 12 张代表地图及官方任务，替换主实验输入；文件清单、校验哈希及引用见 [数据目录](../datasets/movingai/README.md)。

## 推荐结论

优先使用 **Moving AI 的 Room、Maze、Random** 作为算法研究主数据：同时提供地图、标准起终点任务和公开引用，适合替换目前少量自制地图进行重复实验。先选 Room 和宽走廊 Maze，再加入 Random；街道图用于检验不同场景结构。**BARN** 作为机器人狭窄环境补充，其原始地图较小，但需处理格式、车体尺寸和许可证确认。

| 优先级 | 数据集 | 已核实内容 | 对当前项目的用途与接入成本 |
| --- | --- | --- | --- |
| 1 | [Moving AI Room](https://movingai.com/benchmarks/room/index.html) | 512×512，8/16/32/64 房间系列；提供 `.map`、`.scen` 以及 PNG/SVG/PDF | 房间与开口结构适合研究绕行、平滑和采样算法；需保持一格一像素并防止预处理改变障碍 |
| 1 | [Moving AI Maze](https://movingai.com/benchmarks/maze/index.html) | 512×512，多种走廊宽度；提供地图、任务、图像下载 | 适合窄通道与搜索开销比较，先用宽走廊系列；单格走廊对预处理和车体模型很敏感 |
| 1 | [Moving AI Random](https://movingai.com/benchmarks/random/index.html) | 512×512，页面列出 10%–40% 障碍系列及配套任务 | 可分层比较障碍密度、成功率和耗时；高密度任务对采样算法较困难 |
| 2 | [Moving AI Street](https://movingai.com/benchmarks/street/index.html) | 从建筑/道路地图离散化；256×256、512×512、1024×1024，含上海等城市 | 可先用 256×256 检验场景泛化；是算法栅格，不是可直接实车执行的带标定场地图 |
| 2 | [BARN 官方数据集](https://www.cs.utexas.edu/~xiao/BARN/BARN.html) | 300 个预生成导航环境；作者代码说明原始占据栅格为 30×30；含 `.npy`、C-space、`.pgm/.yaml`、Gazebo `.world`、路径和难度指标 | 更接近机器人拥挤障碍导航；先转换原始占据栅格或 PGM，读取原尺度，不能直接沿用 Jackal 的 C-space |

上述 Moving AI 各分类是同一基准库的不同地图族，不应在论文中写成几个互不相关的数据来源。[总入口](https://movingai.com/benchmarks/grids.html)及各分类页均提供下载链接。BARN 官方 ZIP 的 HEAD 请求返回 200，约 4.05 MB：[下载地址](https://www.cs.utexas.edu/~xiao/BARN/BARN_dataset.zip)。

## Moving AI 的格式与实验口径

依据[官方格式说明](https://movingai.com/benchmarks/formats.html)：

- `.map` 开头为 `type octile`、`height`、`width`、`map`，随后为 ASCII 栅格，左上角为 `(0,0)`。
- `.scen` 首行是版本，每个任务含 9 个字段：bucket、地图名、宽、高、起点 x、起点 y、终点 x、终点 y、最优长度。接入本项目时转换为 `start=(start_y,start_x)`、`goal=(goal_y,goal_x)`。
- 标准任务参考长度按对角线代价 `sqrt(2)`、禁止穿墙角计算。当前 A* 和 Dijkstra 为四邻域，PRM/RRT/RRT* 为视线连接，不能直接把结果与 `.scen` 长度相除并称为同一运动模型的最优率。
- 官方还允许任务指定与原图不同的尺寸；初期应只选任务宽高与原图相同的数据，避免无意重采样。
- `.` 和 `G` 为普通可通行地形，`@`、`O`、`T` 为不可通行区域；`S` 和 `W` 有独立地形语义。初期仅选择二值地图族，转换时校验字符集合，遇到其他地形明确拒绝或记录转换策略，不把加权地形静默转成普通空地。

建议从原始 `.map` 确定性转换成黑白 PNG：自由格 255、障碍格 0，每格对应一个像素。官方 PNG 可用于预览，正式实验优先从 `.map` 导出，避免显示配色、放大边框或灰色地形导致二值化语义变化。

旧图片配置的 `default_width=500`、`blur_ksize=5` 会改变 512×512 原图的格点结构和细障碍。现已改为以下基准配置：

```yaml
default_width: 512       # 与当前地图宽度一致；256 图则设置 256
threshold: 127
blur_ksize: 1
obstacle_ratio: 0.5
invert: false
grid_size: 1.0           # 实验约定尺度；不是地图实测米制标定
rectify: false
min_obstacle_area_px: 0
morphology_kernel: 0
```

该配置已作为主实验默认值落地，标准库导入器为 `tools/import_movingai.py`。每次导入应检查生成 grid 与原 `.map` 完全一致，并使用 `--no-drive` 或研究模式。`grid_size=1.0` 会让当前 JSON 输出以米表达约定距离；论文优先比较 `*_length_grid`，明确说明没有真实物理尺度。用于实车时必须另按真实场地尺寸与车体通行条件设计实验。

固定发布版本、地图文件哈希、任务索引、算法参数、随机种子以及原始/后处理指标。不要仅在每张图上任意点击一对起终点；保留目前自制小地图作为快速回归夹具，公开地图用于主实验。

旧 `tools/compare_algorithms.py` 强制缩放到 80×80 并自动移动障碍起终点，已在非真机验收清理中移除。统一使用主 CLI 的研究模式或规划模块 API，显式指定地图配置和标准任务坐标。

## BARN 的适用边界

[作者官方页面](https://www.cs.utexas.edu/~xiao/BARN/BARN.html)说明 BARN 面向整个感知—规划—执行导航系统，强调拥挤障碍、低层运动能力，提供 300 个环境；原套件按 Clearpath Jackal（508×430×250 mm）配置，并提供按不同机器人尺寸重生成的代码。

[作者生成器 README](https://github.com/dperille/jackal-map-creation)进一步说明：

- 原始 world 占据栅格和 C-space 分别存为 `.npy`，C-space 为 30×30；生成器默认 300 个 30×30 环境。
- `map_files` 存放 ROS map_server 使用的 PGM/YAML，`path_files` 为 n×2 路径数组，按 row、col 排列。
- 旧生成器要求 Python 2，不应为读取已有地图而直接引入到本项目 Python 3 环境。

本项目目前从 PNG/JPG/BMP 菜单选图，不直接导入 `.npy`、`.map`、PGM/YAML。BARN 应先建立有来源记录的转换：优先使用原占据 grid，或将 PGM 无损转换 PNG 并读取 YAML 的 `resolution` 和 `origin`，确认是否含未知区域及灰度语义。不要把图像像素坐标与 ROS 世界坐标直接等同。

原 C-space 已考虑其他车型的外形，不能直接声称适合 TurboPi。当前项目尚无车体轮廓/障碍膨胀模块；可以先做离线几何规划比较，但不能用中心线可通行证明小车可通过。

## 许可与论文引用

Moving AI [总页](https://movingai.com/benchmarks/grids.html)声明数据采用 [Open Data Commons Attribution License 1.0](https://opendatacommons.org/licenses/by/1-0/)，并单独说明商业游戏图的授权差异：BioWare 允许研究分发，其他部分游戏地图未获得明确再分发许可。因此优先采用人工生成的 Room/Maze/Random 及来源清楚的 Street，保留下载来源、声明和归属，不把所有游戏素材写成无限制可商用。

Moving AI 官方要求引用：Nathan Sturtevant, *Benchmarks for Grid-Based Pathfinding*, IEEE Transactions on Computational Intelligence and AI in Games, 4(2), 144–148, 2012。

BARN 官方页提供公开下载并要求引用，但本次页面未见明确数据许可证；作者生成器 GitHub 元数据的 `license` 为 `null`。这不能解释为“禁止使用”，也不能推定为 MIT/CC 开放许可；若要把完整数据包随本仓库再分发，应先确认数据许可。另一项目的许可证不能自动套用到 BARN 原始数据。

BARN 官方要求引用：Daniel Perille, Abigail Truong, Xuesu Xiao, Peter Stone, *Benchmarking Metric Ground Navigation*, IEEE SSRR, 2020。[作者论文链接](https://arxiv.org/abs/2008.13315)。

## 本次访问限制

- 已实际访问并核对 Moving AI 总页、格式页、Room/Maze/Random/Street 分类页，以及 UT Austin 的 BARN 官方页和作者生成器 README。
- GMU 的 BARN 旧链接访问超时；使用上文实际可访问的 UT Austin 官方地址。
- Freiburg SLAM 数据页本次 HTTPS 连接失败，未据此推荐其具体可下载地图或许可证。
- 未下载 BARN 完整数据或运行生成器，包内具体分辨率、未知值与文件内容仍需导入前核验。主实验地图已替换为 Room/Maze/Random 子集，未修改规划算法。
