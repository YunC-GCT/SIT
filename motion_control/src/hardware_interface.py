from abc import ABC, abstractmethod

class HardwareInterface(ABC):
    @abstractmethod
    def execute_path(self, deltas, turn_angles):
        """执行路径：输入方向变化序列和转向角序列，阻塞直到完成或出错"""
        pass

    @abstractmethod
    def stop(self):
        """紧急停止"""
        pass