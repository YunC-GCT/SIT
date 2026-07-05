import rclpy
import math
import threading
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
        self._stop_flag = False
        self._stop_event = threading.Event()   # 用于键盘停止
        self.deltas = []
        self.turn_angles = []
        self.index = 0
        self.phase = 'turn'
        self.progress = 0.0
        self.twist = Twist()

    def _keyboard_listener(self):
        """监听键盘输入，按 p 并回车即停止"""
        while not self._stop_event.is_set():
            try:
                key = input()
                if key.strip().lower() == 'p':
                    self.node.get_logger().info("用户按下 p 键，正在停止小车...")
                    self._stop_event.set()
                    break
            except EOFError:
                break

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
        self._stop_flag = False
        self._stop_event.clear()

        # 启动键盘监听线程
        keyboard_thread = threading.Thread(target=self._keyboard_listener, daemon=True)
        keyboard_thread.start()

        self.node.get_logger().info(f"开始跟踪，共 {len(self.deltas)} 段（按 p 并回车可停止）")

        try:
            rclpy.spin(self.node)
        except KeyboardInterrupt:
            self.node.get_logger().info("键盘中断，正在停止...")
        except Exception as e:
            self.node.get_logger().error(f"运行异常: {e}")
        finally:
            self._stop_robot()
            if self.node is not None:
                self.node.destroy_node()
            rclpy.shutdown()
            print("[Ros2Interface] 节点已关闭，小车已停止。")

    def _timer_callback(self):
        # 检查键盘停止信号
        if self._stop_event.is_set():
            self._stop_robot()
            return

        if self._stop_flag or self.index >= len(self.deltas):
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
        if self.pub is not None:
            self.pub.publish(Twist())
        if self.timer is not None and not self.timer.is_canceled():
            self.timer.cancel()
        if self.node is not None:
            self.node.get_logger().info("路径完成或中断，小车已停止。")