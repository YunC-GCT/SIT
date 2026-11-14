# motion_control — TurboPi turn 运动控制

加载 `path_postprocess` 的统一路径段 JSON，按 “转向 -> 零速过渡 -> 前进 -> 停止”
向 TurboPi 底盘发布 `geometry_msgs/msg/Twist`。速度话题使用绝对 `/cmd_vel`。

## 快速开始

```bash
python motion_control/run_follower.py \
  --postprocess-result runs/<run-id>/path_postprocess/result.json
```

在实车上运行前，先进入 `turbopi` 容器，再在容器内项目目录执行命令。不要把
Windows 开发机路径当作实车运行路径。

## 执行语义

- 初始车头默认是地图正北，地图上方为 north。
- 每个路径段使用 `direction_deg` 计算最短转角。
- 180 度掉头拆成两个 90 度原地转向，中间插入零速过渡。
- 转向发布 `angular.z`，前进只发布 `linear.x`，不使用默认漂移或横移。
- 当前执行器始终使用计时开环，没有 `/odom` 订阅；`odom_topic` 仅为保留参数。前进时长按 `distance_m / empirical_forward_speed_m_s` 估算，转向时长按转角与 `turn_speed` 估算。
- 前进发布时长按控制周期量化，舍入余量在段间累计；不能据此保证每个短段都能精确到位。

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

CLI 也处理 `SIGTERM` 和平台支持的 `SIGHUP`。强制结束进程或断电无法保证运行退出处理。代码中的 `stopped` 表示重复零速发布流程完成，实际车轮停止需要实车观察。

只发布停车命令可在容器内项目根目录执行 `python -m motion_control.run_follower --stop-only`。该入口不写运动执行结果 JSON；命令返回成功也不能代替现场停车确认。

## 参数与预计时间

控制参数以 `config.yaml` 为准：当前 `move_speed=0.5` 是前进命令，`empirical_forward_speed_m_s=0.30` 是临时经验速度，两者含义不同。转向默认 `turn_speed=1.0 rad/s`，尚无实车旋转速度标定依据。

后处理默认 `turn_speed_rad_s=2.0`，预览时长使用后处理参数，执行器使用运动配置并重新计算转角和持续时间，且包含额外零速过渡。因此预览时长不是实际执行时间；比较实验时应记录两份配置，并保持初始朝向一致。

多算法研究结果的路径段位于 `outputs.comparisons[*].outputs.segments`，控制器仅接受顶层 `outputs.segments`。需要实车运行时，应针对选定算法单独规划和后处理后再执行。
