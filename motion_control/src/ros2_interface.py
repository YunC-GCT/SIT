import rclpy
import math
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist

DIR_NORTH, DIR_EAST, DIR_SOUTH, DIR_WEST = 0, 1, 2, 3

def delta_to_dir(dr, dc):
    if dr == -1 and dc == 0: return DIR_NORTH
    if dr == 0 and dc == 1: return DIR_EAST
    if dr == 1 and dc == 0: return DIR_SOUTH
    if dr == 0 and dc == -1: return DIR_WEST
    return DIR_NORTH

def compute_turn_angles(directions):
    if not directions:
        return []
    current = directions[0]
    angles = [0]
    for i in range(1, len(directions)):
        target = directions[i]
        diff = (target - current) % 4
        if diff == 0:   turn = 0
        elif diff == 1: turn = -90
        elif diff == 2: turn = 180
        else:          turn = 90
        angles.append(turn)
        current = target
    return angles

class Ros2PathExecutor:
    def __init__(self, turn_speed=2.0, move_speed=0.3, grid_size=0.3, timer_period=0.1):
        self.turn_speed = turn_speed
        self.move_speed = move_speed
        self.grid_size = grid_size
        self.timer_period = timer_period
        self.node = None
        self.timer = None
        self.pub = None
        self.deltas = []
        self.turn_angles = []
        self.index = 0
        self.phase = 'turn'
        self.progress = 0.0
        self.twist = Twist()

    def execute_path(self, deltas, turn_angles):
        rclpy.init(args=None)
        self.node = Node('path_follower')
        self.pub = self.node.create_publisher(Twist, 'cmd_vel', 1)
        self.timer = self.node.create_timer(self.timer_period, self._timer_callback)

        self.deltas = deltas
        self.turn_angles = turn_angles
        self.index = 0
        self.phase = 'turn'
        self.progress = 0.0
        self.twist = Twist()

        self.node.get_logger().info(f"开始跟踪，共 {len(self.deltas)} 段（按 Ctrl+C 退出,退出前会自动停车）")

        try:
            rclpy.spin(self.node)
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

    def _timer_callback(self):
        if self.index >= len(self.deltas):
            self._stop_robot()
            return

        target_angle = abs(self.turn_angles[self.index])

        if self.phase == 'turn':
            sign = 1.0 if self.turn_angles[self.index] >= 0 else -1.0
            self.twist.linear.x = 0.0
            self.twist.linear.y = 0.0
            self.twist.angular.z = sign * self.turn_speed
            self.progress += self.turn_speed * (180.0 / math.pi) * self.timer_period
            if self.progress >= target_angle - 1.0:
                self.phase = 'move'
                self.progress = 0.0
                self.twist.angular.z = 0.0

        elif self.phase == 'move':
            self.twist.linear.x = self.move_speed
            self.twist.linear.y = 0.0
            self.twist.angular.z = 0.0
            self.progress += self.move_speed * self.timer_period
            if self.progress >= self.grid_size - 0.01:
                self.progress = 0.0
                self.index += 1
                self.phase = 'turn'

        self.pub.publish(self.twist)
        self.node.get_logger().info(
            f'段 {self.index+1}/{len(self.deltas)} '
            f'阶段: {self.phase} '
            f'线速度: ({self.twist.linear.x:.2f}, {self.twist.linear.y:.2f}) '
            f'角速度: {self.twist.angular.z:.2f} '
            f'进度: {self.progress:.2f}'
        )

    def _stop_robot(self):
        """
        真正让小车停下来:取消控制循环 → 连续发多次零速度 Twist →
        给底层 DDS 一小段时间把消息送出去,最后再 destroy 节点。
        多次发送是为了确保至少一帧被电机驱动订阅端收到,
        避免 destroy 太快导致唯一一帧零速度消息被丢弃。
        """
        if self.timer is not None and not self.timer.is_canceled():
            self.timer.cancel()
        if self.pub is not None:
            zero = Twist()
            for _ in range(10):
                self.pub.publish(zero)
                time.sleep(0.02)
            time.sleep(0.1)  # 给 DDS 一点时间把消息送出去
        if self.node is not None:
            self.node.get_logger().info("小车已停止（Twist 零速度已发出）。")