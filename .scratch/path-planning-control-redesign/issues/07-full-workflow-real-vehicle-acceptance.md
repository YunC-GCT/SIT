# 07 — 全流程验证与 TurboPi 实车验收

**What to build:** 在固定实验输入上验证完整模块链路和恢复行为，并在隔离其他速度发布者后，以低速完成 TurboPi 的方向、转向、路径跟随和安全停车验收。

Blocked by: 06

Status: ready-for-human

- [x] 固定地图、起终点和随机种子完成建图到模拟控制的完整集成运行
- [x] 验证每个阶段的 JSON、manifest、日志和失败状态可追踪
- [x] 验证普通模式和研究模式的交互数量与行为符合规格
- [ ] 停止或隔离其他 `/cmd_vel` 发布者后，确认底盘订阅者为 `/mecanum_chassis_node`
- [ ] 低速验证前进、后退、左转、右转、90 度转向和路径段跟随
- [x] 验证路径完成、异常和 Ctrl+C 后小车先停车再退出
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
