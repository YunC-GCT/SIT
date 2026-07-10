# motion_control — 运动控制

> 加载路径 → 麦轮全向平移 → ROS2 `/cmd_vel`。odom 闭环段切换，安全停车。

---

## 快速开始

```bash
# 确保 data/input/ 下有 path_data.npy + grid_map.npz（main.py 自动复制）
python run_follower.py
```

---

## 麦轮全向循迹

**不转弯**。利用麦克纳姆轮全向平移，每段直接用 `(vx, vy)` 组合，`angular.z=0`。小车默认朝 **EAST**。

| 栅格方向 | 相对小车朝向 | 速度指令 | 含义 |
|----------|:---------:|---------|------|
| EAST | 同向 | `(+v, 0)` | 前进 |
| SOUTH | 右侧 90° | `(0, -v)` | 右平移 |
| WEST | 反向 | `(-v, 0)` | 后退 |
| NORTH | 左侧 90° | `(0, +v)` | 左平移 |

```
        NORTH (0,+v) 左平移
            ▲
WEST (-v,0) ◄─── 小车朝 EAST ───► EAST (+v,0) 前进
            ▼
        SOUTH (0,-v) 右平移
```

```python
from src.ros2_interface import compute_vel_commands, delta_to_dir

deltas = [(0,1), (1,0), (0,1)]                      # 东→南→东
dirs   = [delta_to_dir(dr,dc) for dr,dc in deltas]   # [EAST, SOUTH, EAST]
cmds   = compute_vel_commands(dirs, initial_heading='east', move_speed=0.3)
# → [(0.0, -0.3), (-0.3, 0.0), (0.0, -0.3)]
```

---

## odom 闭环段切换

| 条件 | 行为 |
|------|------|
| odom 正常 | 位移 ≥ `grid_size-0.01m` → 切段 |
| odom 中断 | timer 累加兜底 → 1.5×超时强制切段 |
| 连续 30 段无 odom | warn：检查 `/odom` 话题 |

日志中 `实际 0.298m` = odom 闭环生效；`计时 0.05m` = 退化模式。

---

## grid_size 校验

启动时从 `grid_map.npz` 读取地图创建时的物理尺度 vs `config.yaml`：

```
[Follower] grid_size 校验通过: 0.3000m
# 不一致时：
[Follower] ⚠️ grid_size 不一致！地图: 0.2500m, config: 0.3000m。已自动使用地图值。
```

---

## 安全停车

Ctrl+C → `_stop_robot()` → **10 次零速度 Twist**（间隔 20ms）→ 100ms DDS 缓冲 → destroy。确保零速度送达电机。

---

## 依赖注入（仿真基础）

```python
executor = Ros2PathExecutor(move_speed=0.3, grid_size=0.3, ...)

# 正常模式：自动 setup_ros2()
executor.execute_path(deltas, vel_commands)

# 高级模式：调用方管理 ROS2 生命周期
node, pub = executor.setup_ros2()
executor.execute_path(deltas, vel_commands, node=node, pub=pub)

# 仿真模式（未来）：注入 FakePublisher
executor.pub = FakePublisher()
executor.execute_path(deltas, vel_commands)
```

---

## config.yaml

```yaml
motion_control:
  grid_size: 0.3          # m/格（启动校验 vs npz）
  move_speed: 0.3         # 线速度 m/s
  timer_period: 0.1       # 控制周期 s
  odom_topic: "/odom"     # 里程计话题（BEST_EFFORT）
  initial_heading: "east"# 小车初始朝向
```

---

## 调试

```bash
ros2 topic echo /cmd_vel                      # 看速度指令
ros2 topic echo /odom --field pose.pose.position  # 看里程计
ros2 topic hz /odom                           # 看频率
```

## 排障

| 问题 | 解决 |
|------|------|
| 横移反了 | `compute_vel_commands()` 中 `rel==1`/`rel==3` 符号对调 |
| grid_size warn | 同步 config.yaml 或重建地图 |
| 无 odom | `ros2 topic hz /odom`；改 `config.yaml` 的 `odom_topic` |
| Ctrl+C 不停 | 确认 `_stop_robot()` 发 10 次零速度（最新版） |
