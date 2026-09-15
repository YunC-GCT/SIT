# motion_control — TurboPi turn 运动控制

加载 `path_postprocess` 的统一路径段 JSON，按 “转向 -> 零速过渡 -> 前进 -> 停止”
向 TurboPi 底盘发布 `geometry_msgs/msg/Twist`。速度话题使用绝对 `/cmd_vel`。

## 快速开始

```bash
python motion_control/run_follower.py \
  --postprocess-result runs/<run-id>/path_postprocess/result.json \
  --yes
```

在实车上运行前，先进入 `turbopi` 容器，再在容器内项目目录执行命令。不要把
Windows 开发机路径当作实车运行路径。

## 执行语义

- 初始车头默认是地图正北，地图上方为 north。
- 每个路径段使用 `direction_deg` 计算最短转角。
- 180 度掉头拆成两个 90 度原地转向，中间插入零速过渡。
- 转向发布 `angular.z`，前进只发布 `linear.x`，不使用默认漂移或横移。
- 当前实车没有 `/odom`，段距离默认用经验速度和 timer 开环估算，日志会明确提示。

## 安全预检

启动运动前会检查 `/cmd_vel`：

- 必须看到 `/mecanum_chassis_node` 订阅 `/cmd_vel`。
- 如果存在其他 `/cmd_vel` 发布者，默认阻止启动。
- 预检信息展示后，需要终端输入 `yes`，或显式传入 `--yes`，才会启动运动。
- 确认其他发布者已安全隔离后，才使用 `--allow-other-publishers`。

调试假 ROS 图时可用 `--no-require-chassis-subscriber`，不要在实车默认流程里使用。

执行成功或失败都会写入 `motion_control` 的结构化 `result.json`。默认位置是
`runs/<run-id>/motion_control/result.json`，也可用 `--output` 指定。

## 安全停车

完成、异常、`Ctrl+C` 和关闭流程都会先重复发布零速度 `Twist`，等待通信缓冲，
再销毁节点并幂等关闭 ROS2。重复调用关闭逻辑仍会先发送停车命令。
