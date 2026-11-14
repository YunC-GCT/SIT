# 项目文档目录

此目录面向项目使用者、开发者及实验评审。当前分支保留程序、测试、地图数据、HTML 报告及阅读文档；不包含 Agent 工作规则和内部任务票据。

## 使用与展示

- [程序介绍与使用手册](程序介绍与使用手册.md)：运行方式、栅格选择、交互与故障恢复。
- [项目设计报告](project-design-report.html)：系统架构、模块职责和优化方向。
- [交互形式报告](interaction-report.html)：选图、选点、预览和恢复菜单。
- [验收流程](acceptance.md)：离线验收和实车记录方法。
- [硬件与部署参考](turbopi-hardware.md)：树莓派 Docker 环境、计时开环限制及安全停车。

HTML 报告使用浏览器本地打开；GitHub 文件页面展示源代码。请保留同目录的 report-diagrams.css 和图片资源。

## 开发参考

- [接口清单](../API_INDEX.md)与[领域词汇](../CONTEXT.md)。
- [架构决策](adr/)：模块边界、统一 JSON、可替换平滑器与交互安全确认。
- 模块说明：[建图](../map_pipeline/README.md)、[规划](../path_planner/README.md)、[后处理](../path_postprocess/README.md)、[运动控制](../motion_control/README.md)。

## 地图与实验

- [公开数据集选型](public-map-datasets.md)和[地图来源说明](../datasets/movingai/README.md)。
- [实验结果总览](verification/README.md)：区分最新结果、历史调参和软件验证记录。
- 实验 JSON 保留任务、参数与结果，供复测工具读取；它们不是临时运行日志。

程序运行时仍会在本地生成 runs/，该目录已被 Git 忽略。测试夹具、配置文件和地图原始数据属于可复现程序的一部分，继续保留。
