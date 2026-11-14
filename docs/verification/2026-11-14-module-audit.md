# 模块边界与死代码检查

检查 API/CLI 调度、地图→规划→后处理 JSON、研究结果聚合和现有 Fake ROS2 测试。未修改硬件控制或生命周期代码。

## 修复

- 研究比较条目补传 search_diagnostics，此前单算法中的重试/首解诊断会在聚合时丢失。
- 规划 API 在返回 success 前检查路径相邻节点间的双向碰撞，避免只有节点合法、跨障碍连线却被标记成功；后处理仍独立复核。
- 拒绝非 0/1 地图、NaN/Infinity 地图尺度和步长/连边长度/搜索半径，避免无效上游输入进入算法阶段。

## 清理

已删除用户指定的 generate_test_maps.py、draw_direction_ref.py，已有测试夹具与参考图保留。删除空且无引用的 map_pipeline/run_batch.py、无调用的 path_visualizer.show，移除 API 索引对应条目。

保留 run_planner、planner_pipeline、path_postprocessor、contracts 等稳定或兼容入口；测试和现有调用依赖这些边界。预览绘图、GUI 后端、ROS2 适配与实验复现工具有实际用途，不能按“主流程未直接调用”认定死代码。

## 验证

- 完整 unittest 153 项通过（PYTHONUTF8=1），包括真实子进程链路及 Fake ROS2 正常/异常退出验收。
- 额外构造 12×12 自由地图，真实规划 CLI 一次运行六算法（含 rrt_adaptive），随后后处理 CLI 全部成功。使用临时目录，结果和日志自动清理。
- pyflakes 全源码、工具、测试通过；git diff --check 通过。

本轮未重跑全地图性能实验，未做真实 ROS2/ARM/车辆验收；历史随机任务成功率结论不因此提升。没有发现现有测试覆盖范围内新的调用链阻断，但不能据此保证所有异常或部署环境没有问题。修改尚未提交。
