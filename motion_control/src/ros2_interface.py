import rclpy
import math
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

DIR_NORTH, DIR_EAST, DIR_SOUTH, DIR_WEST = 0, 1, 2, 3

# 段切换兜底超时倍数:理论耗时 = grid_size / move_speed,
# timer 超时 1.5 倍还没到位就强制切下一段(典型场景:odom 还没发第一帧 / 小车卡住)。
SEGMENT_TIMEOUT_RATIO = 1.5

# 没收到 odom 时,退化用 timer 走多少段后报警(避免 silent 退化)。
ODOM_MISSING_WARN_AFTER = 30  # 段

# initial_heading 字符串 → 方向代号
HEADING_TO_DIR = {
    'north': DIR_NORTH,
    'east': DIR_EAST,
    'south': DIR_SOUTH,
    'west': DIR_WEST,
    'n': DIR_NORTH, 'e': DIR_EAST, 's': DIR_SOUTH, 'w': DIR_WEST,
}


def _diff_to_turn(diff):
    """把 (target - current) % 4 映射成 4 个离散转向角。
    diff=0 直行 / 1 右转 -90°(从 dir 编码看是逆时针,实际转角按实测可调) /
    diff=2 掉头 180° / diff=3 左转 +90°
    """
    if diff == 0:   return 0
    if diff == 1:   return -90
    if diff == 2:   return 180
    return 90


def delta_to_dir(dr, dc):
    if dr == -1 and dc == 0: return DIR_NORTH
    if dr == 0 and dc == 1: return DIR_EAST
    if dr == 1 and dc == 0: return DIR_SOUTH
    if dr == 0 and dc == -1: return DIR_WEST
    return DIR_NORTH


def compute_turn_angles(directions, initial_heading='east'):
    """
    根据方向序列和初始朝向,计算每段转向角(度)。

    - 第一个转向角:从 initial_heading 转到 directions[0] 的差分
    - 后续:从 directions[i-1] 转到 directions[i] 的差分
    - 返回列表长度 = len(directions),angles[i] 是进入第 i 段前的转向角
      (其中 angles[0] 是从初始朝向转第一段方向,即使 directions[0] == init 也返回 0)
    """
    if not directions:
        return []
    init = HEADING_TO_DIR.get(initial_heading.lower() if isinstance(initial_heading, str)
                              else initial_heading, DIR_NORTH)
    angles = [_diff_to_turn((directions[0] - init) % 4)]
    current = directions[0]
    for i in range(1, len(directions)):
        target = directions[i]
        angles.append(_diff_to_turn((target - current) % 4))
        current = target
    return angles


def compute_vel_commands(directions, initial_heading='east', move_speed=0.3):
    """
    麦轮全向模式：将方向序列直接映射为机器人坐标系下的 (vx, vy) 速度指令。
    麦轮无需原地旋转——利用全向平移能力直接朝目标方向移动。

    映射规则（机器人坐标系 REP 103: x 前, y 左）：
    - 与初始朝向相同 → (+move_speed, 0) 前进
    - 与初始朝向相反 → (-move_speed, 0) 后退
    - 初始朝向右侧 90° → (0, -move_speed) 右平移
    - 初始朝向左侧 90° → (0, +move_speed) 左平移

    :param directions: 方向代号列表 [DIR_NORTH, DIR_EAST, ...]
    :param initial_heading: 小车初始朝向字符串,决定机器人的"前方"对应哪个栅格方向
    :param move_speed: 线速度 m/s
    :return: [(vx, vy), ...] 长度 = len(directions)
    """
    if not directions:
        return []
    init = HEADING_TO_DIR.get(
        initial_heading.lower() if isinstance(initial_heading, str) else initial_heading,
        DIR_NORTH)
    commands = []
    for d in directions:
        rel = (d - init) % 4
        if rel == 0:       # 前进（与初始朝向同向）
            commands.append((move_speed, 0.0))
        elif rel == 1:     # 右平移（初始朝向右侧）
            commands.append((0.0, -move_speed))
        elif rel == 2:     # 后退（与初始朝向反向）
            commands.append((-move_speed, 0.0))
        else:              # rel == 3: 左平移（初始朝向左侧）
            commands.append((0.0, move_speed))
    return commands


def estimate_runtime(turn_angles, grid_size, turn_speed, move_speed):
    """
    估算总运行时间(秒)。
    - 转弯耗时 = Σ|角度(度)| / (turn_speed rad/s 转 deg/s)
    - 直行耗时 = 段数 * grid_size / move_speed
    注意:这是 open-loop 下的理论值,实际由 odom 闭环驱动。
    """
    if not turn_angles:
        return 0.0
    turn_deg_per_s = turn_speed * (180.0 / math.pi)
    turn_time = sum(abs(a) for a in turn_angles) / turn_deg_per_s
    move_time = len(turn_angles) * grid_size / move_speed
    return turn_time + move_time


class Ros2PathExecutor:
    def __init__(self, turn_speed=2.0, move_speed=0.3, grid_size=0.3,
                 timer_period=0.1, odom_topic='/odom', initial_heading='east'):
        self.turn_speed = turn_speed
        self.move_speed = move_speed
        self.grid_size = grid_size
        self.timer_period = timer_period
        self.odom_topic = odom_topic
        self.initial_heading = initial_heading
        self.node = None
        self.timer = None
        self.pub = None
        self.odom_sub = None
        self.deltas = []
        self.vel_commands = []    # 麦轮全向模式：每段的 (vx, vy) 速度指令
        self.index = 0
        self.progress = 0.0       # timer 累加进度（odom 不可用时退化用）
        self.twist = Twist()
        self.done = False         # 走完所有段时由 _timer_callback 置 True,自旋循环据此退出

        # odom 闭环状态
        self.current_xy = None          # 最新 odom 位姿 (x, y)
        self.segment_start_xy = None    # 当前段起点的 odom 位姿
        self.segment_start_time = 0.0   # 当前段起始时刻（用于 timer 兜底超时）
        self.odom_received = False      # 是否收过至少一帧 odom
        self.segments_without_odom = 0  # 连续没 odom 的段数(用于报警)

    def setup_ros2(self):
        """创建 ROS2 节点、发布者、订阅者、定时器。
        调用方也可以自行创建 Node 后直接传给 execute_path(node=self.node, pub=self.pub)。
        """
        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = Node('path_follower')
        self.pub = self.node.create_publisher(Twist, 'cmd_vel', 1)
        self.timer = self.node.create_timer(self.timer_period, self._timer_callback)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odom_sub = self.node.create_subscription(
            Odometry, self.odom_topic, self._odom_callback, qos)
        return self.node, self.pub

    def execute_path(self, deltas, vel_commands, node=None, pub=None):
        """
        麦轮全向循迹：每段直接用 (vx, vy) 平移，无需原地转向。

        :param deltas: 栅格位移列表 [(dr, dc), ...]
        :param vel_commands: 每段的速度指令 [(vx, vy), ...]，长度 = len(deltas)
        :param node: 可选，已创建好的 ROS2 Node（用于复用/仿真注入）
        :param pub:  可选，已创建好的 cmd_vel Publisher
        """
        if node is not None:
            self.node = node
        if pub is not None:
            self.pub = pub
        if self.node is None or self.pub is None:
            self.setup_ros2()

        self.deltas = deltas
        self.vel_commands = vel_commands
        self.index = 0
        self.progress = 0.0
        self.twist = Twist()
        self.done = False

        # 重置 odom 状态
        self.current_xy = None
        self.segment_start_xy = None
        self.segment_start_time = time.monotonic()
        self.odom_received = False
        self.segments_without_odom = 0

        n = len(self.deltas)
        move_time = n * self.grid_size / self.move_speed

        self.node.get_logger().info(
            f"开始麦轮全向循迹，共 {n} 段（按 Ctrl+C 退出,退出前会自动停车）")
        self.node.get_logger().info(
            f"初始朝向: {self.initial_heading}（麦轮全向模式，无转向阶段，每段直接平移）")
        self.node.get_logger().info(
            f"预估运行 {move_time:.1f}s（纯平移 @ {self.move_speed:.2f} m/s, "
            f"grid={self.grid_size}m）")
        self.node.get_logger().info(
            f"odom 闭环:订阅 {self.odom_topic} (BEST_EFFORT),"
            f"段切换由位移驱动,timer 兜底 {SEGMENT_TIMEOUT_RATIO}x 超时")

        try:
            # 自旋循环:走完路径(_timer_callback 置 done=True)或 Ctrl+C 退出
            # 比 rclpy.spin() 好的地方:走完路径后能正常退出,主进程 wait() 能返回,
            # 外层 main.py 循环才能继续;Ctrl+C 行为不变(KeyboardInterrupt 仍能截到)
            while rclpy.ok() and not self.done:
                rclpy.spin_once(self.node, timeout_sec=self.timer_period)
        except KeyboardInterrupt:
            self.node.get_logger().info("收到 Ctrl+C，准备先停车再退出程序...")
        except Exception as e:
            self.node.get_logger().error(f"运行异常: {e}")
        finally:
            # 退出程序前先把小车停下,这是 Ctrl+C 流程的核心
            self._stop_robot()
            if self.node is not None:
                self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            print("[Ros2Interface] 小车已停止，程序退出。")

    def _odom_callback(self, msg: Odometry):
        """只更新位姿缓存,不做控制决策(决策统一在 timer 里做)。"""
        pos = msg.pose.pose.position
        self.current_xy = (pos.x, pos.y)
        self.odom_received = True
        # 当前段运行时如果 odom 还没到,segment_start_xy 是 None;
        # odom 一旦到达,立即锚定,避免切段时起点错位
        if self.segment_start_xy is None:
            self.segment_start_xy = self.current_xy

    def _timer_callback(self):
        """
        麦轮全向循迹回调：每段直接用 (vx, vy) 平移，无转向阶段。
        段切换优先 odom 位移驱动，odom 不可用时退化到 timer 累加。
        """
        if self.index >= len(self.deltas):
            self._stop_robot()
            self.done = True
            return

        # 设置当前段的速度指令（麦轮直接平移，不旋转）
        vx, vy = self.vel_commands[self.index]
        self.twist.linear.x = vx
        self.twist.linear.y = vy
        self.twist.angular.z = 0.0

        # 优先用 odom 位移判断段切换;没 odom 时退化到 timer 累加
        actual_progress = self._compute_move_progress()

        # timer 兜底:理论耗时 1.5 倍还没到,强制切段 + warn
        elapsed = time.monotonic() - self.segment_start_time
        timeout_s = (self.grid_size / self.move_speed) * SEGMENT_TIMEOUT_RATIO
        timed_out = elapsed > timeout_s

        if actual_progress >= self.grid_size - 0.01 or timed_out:
            if timed_out and actual_progress < self.grid_size - 0.01:
                if self.odom_received:
                    self.node.get_logger().warn(
                        f'段 {self.index+1}/{len(self.deltas)} '
                        f'timer 兜底触发 (elapsed {elapsed:.2f}s > {timeout_s:.2f}s, '
                        f'odom 位移 {actual_progress:.3f}m 未到 {self.grid_size}m)')
                else:
                    self.segments_without_odom += 1
                    if self.segments_without_odom >= ODOM_MISSING_WARN_AFTER:
                        self.node.get_logger().warn(
                            f'连续 {self.segments_without_odom} 段未收到 odom,'
                            f'检查 {self.odom_topic} 话题是否在发')
            else:
                self.segments_without_odom = 0

            # 切下一段
            self.progress = 0.0
            self.segment_start_xy = self.current_xy
            self.segment_start_time = time.monotonic()
            self.index += 1

        self.pub.publish(self.twist)
        actual_log = (f'实际 {actual_progress:.3f}m'
                      if self.odom_received and self.segment_start_xy is not None
                      else f'计时 {self.progress:.2f}m')
        self.node.get_logger().info(
            f'段 {self.index+1}/{len(self.deltas)} '
            f'线速度: ({self.twist.linear.x:.2f}, {self.twist.linear.y:.2f}) '
            f'角速度: {self.twist.angular.z:.2f} '
            f'进度: {actual_log}')

    def _compute_move_progress(self):
        """计算当前段已走的位移(米)。
        优先 odom 位移;没有 odom / 没收到第一帧时,退化用 timer 累加。
        注意：退化路径会递增 self.progress，每次回调只应调用一次本方法。
        """
        if (self.odom_received
                and self.current_xy is not None
                and self.segment_start_xy is not None):
            dx = self.current_xy[0] - self.segment_start_xy[0]
            dy = self.current_xy[1] - self.segment_start_xy[1]
            return (dx * dx + dy * dy) ** 0.5
        # 退化路径:用 timer 累加（修复：之前 progress 永远是 0）
        self.progress += self.move_speed * self.timer_period
        return self.progress

    def _stop_robot(self):
        """
        真正让小车停下来:取消控制循环 → 连续发多次零速度 Twist →
        给底层 DDS 一小段时间把消息送出去,最后再 destroy 节点。
        多次发送是为了确保至少一帧被电机驱动订阅端收到,
        避免 destroy 太快导致唯一一帧零速度消息被丢弃。
        """
        if self.timer is not None and not self.timer.is_canceled():
            self.timer.cancel()
        if self.odom_sub is not None:
            self.node.destroy_subscription(self.odom_sub)
            self.odom_sub = None
        if self.pub is not None:
            zero = Twist()
            for _ in range(10):
                self.pub.publish(zero)
                time.sleep(0.02)
            time.sleep(0.1)  # 给 DDS 一点时间把消息送出去
        if self.node is not None:
            self.node.get_logger().info("小车已停止（Twist 零速度已发出）。")