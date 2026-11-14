# 07 — 全流程验证与 TurboPi 实车验收

**What to build:** 在固定实验输入上验证完整模块链路和恢复行为，并在隔离其他速度发布者后，以低速完成 TurboPi 的方向、转向、路径跟随和安全停车验收。

Blocked by: 06

Status: ready-for-human

- [x] 固定地图、起终点和随机种子完成建图到模拟控制的完整集成运行
- [x] 验证每个阶段的 JSON、manifest、日志和失败状态可追踪
- [x] 验证普通模式和研究模式的交互数量与行为符合规格
- [ ] 停止或隔离其他 `/cmd_vel` 发布者后，确认底盘订阅者为 `/mecanum_chassis_node`
- [ ] 低速验证前进、后退、左转、右转、90 度转向和路径段跟随
- [x] 通过 Fake ROS2 验证路径完成、异常和 Ctrl+C 后零速度发布先于退出
- [ ] 在真实底盘上观察并记录路径完成、异常和 Ctrl+C 后停车及退出结果
- [ ] 记录实车方向、经验速度、运行条件和验收结果，不宣称厘米级精度

## Answer

Implemented the repeatable software acceptance path:

- `tools/acceptance.py` runs the fixed map, start/goal, and seed through the
  real module CLIs, then executes the real motion executor against a Fake ROS2
  graph.
- `motion_control.src.simulation` records `/cmd_vel` messages, preflight
  endpoints, virtual elapsed time, completion, repeated zero-speed stop, and
  node destruction.
- `acceptance/result.json` and `acceptance/manual_record.json` are stored in
  the same run batch. Map-stage failures now also retain a module log.
- Real-vehicle direction, condition, and empirical-speed checks remain
  `ready-for-human` because they require the Raspberry Pi and TurboPi hardware.

See `docs/acceptance.md` for the commands and the real-vehicle recording
boundary.

## Comments

2026-11-14 文档核对：将 Fake ROS2 的零速度发布验证与实车停车观察分开记录。现有软件验收记录不能证明真实底盘已完成停车验收；人工条件、方向、速度和停车记录仍需补齐。

2026-11-14 非真机复验：136/136 软件测试通过，核心包语句覆盖率 85.98%，pyflakes 无警告。12 张公开图逐格及哈希校验通过，12 个官方任务的 A*/Dijkstra 真实 CLI 流水线成功。普通/研究模式 Fake ROS2 各 8 项检查通过。另测三张图的 PRM/RRT/RRT* 共 9 次，默认参数与 seed 42 均返回结构化 NO_PATH，下游未启动；不得表述为五算法公开地图全部通过。完整记录见 [非真机验收](../../../docs/verification/2026-11-14-offline.md)。实车待办不变。
