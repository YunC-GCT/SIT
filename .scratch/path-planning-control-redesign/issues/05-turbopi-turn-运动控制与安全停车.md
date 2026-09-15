# 05 — TurboPi turn 运动控制与安全停车

**What to build:** 运动控制器读取统一路径段并驱动 TurboPi：每段先原地转向，再零速过渡，再沿车头方向前进；无 `/odom` 时使用经验速度和 timer 估算，并在所有退出路径安全停车。

**Blocked by:** 01、04

**Status:** resolved

- [x] 发布绝对话题 `/cmd_vel` 的 `geometry_msgs/msg/Twist`
- [x] 采用地图正北作为初始车头，正确计算最短转角和 180 度拆分
- [x] 按“旋转 → 零速过渡 → 前进 → 停止”执行每个路径段
- [x] 无 `/odom` 时使用约 `0.30 m/s` 的可配置经验速度和 timer 估算，并明确报告开环
- [x] 启动前检查底盘订阅者和其他 `/cmd_vel` 发布者，未确认时不驱动
- [x] 使用 Fake ROS2 发布器测试 Twist 内容、状态顺序、超时和停车
- [x] Ctrl+C、异常、完成和节点销毁都先重复发布零速度，再幂等关闭 ROS2

## Answer

运动控制已切换为读取 `path_postprocess` 的统一路径段 JSON，并使用 turn 模式执行：
每段按路径段方向从地图正北初始车头计算最短转角，180 度掉头拆为两个 90 度转向，
随后插入零速过渡、沿 `linear.x` 前进并停车。ROS2 发布器使用绝对 `/cmd_vel`。

无 `/odom` 时执行器明确记录 timer 开环估算，默认经验前进速度为 `0.30 m/s`，
前进命令值和经验速度均可通过 `motion_control/config.yaml` 配置。启动前会检查
`/cmd_vel` 的底盘订阅者和其他发布者；未看到 `/mecanum_chassis_node` 或发现未确认的
其他发布者时，不启动运动。预检结果会先展示给用户，只有交互输入 `yes` 或显式使用
`--yes` 后才启动运动。

安全退出路径统一为重复发布零速度、留出通信缓冲、销毁节点并幂等关闭 ROS2。
ROS 图查询失败同样会阻断运动，CLI 会将输入、阶段、原因和建议动作写入结构化
`motion_control/result.json`。Fake ROS2 测试覆盖了 Twist 内容、turn/zero/forward
顺序、timer 时长、预检阻断、图查询失败和幂等关闭。
