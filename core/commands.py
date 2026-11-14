"""顶层调度器使用的模块 CLI 命令构造。"""

from __future__ import annotations

import json
import os
import sys
from typing import Callable
from .batch import RunBatch


class CommandMixin:
    """为 PipelineOrchestrator 提供各模块的命令构造方法。"""

    batch: RunBatch
    image_path: str | None
    start: tuple[int, int] | None
    goal: tuple[int, int] | None
    terminal_select: bool
    headless: bool
    rectify: bool
    corners: str | None
    arena_width_m: float | None
    arena_height_m: float | None
    rectify_output_width: int | None
    grid_size_m: float | None
    map_config: str | None
    planner_config: str | None
    postprocess_config: str | None
    motion_config: str | None
    mode: str
    algorithms: list[str]
    algorithm: str | None
    seed: int | None
    parameters: dict
    postprocess_parameters: dict
    confirm_drive: bool

    def _command_for(self, module: str) -> Callable[[str], list[str]]:
        """按模块标识返回命令构造函数，未知标识由字典查询抛出 KeyError。"""
        return {
            "map_pipeline": self._map_command,
            "path_planner": self._planner_command,
            "path_postprocess": self._postprocess_command,
        }[module]

    def _common_args(self, output_path: str) -> list[str]:
        """构造各模块共用的结果路径、批次根目录及 run_id 命令行参数。"""
        return [
            "--output",
            output_path,
            "--run-root",
            self.batch.root,
            "--run-id",
            self.batch.run_id,
        ]

    def _map_command(self, output_path: str) -> list[str]:
        """构造建图命令，传递完整起终点或交互选点方式以及地图尺度配置。"""
        command = [
            sys.executable,
            "-m",
            "map_pipeline.json_pipeline",
            self.image_path or "",
        ]
        if self.start is not None and self.goal is not None:
            command += ["--start", f"{self.start[0]},{self.start[1]}"]
            command += ["--goal", f"{self.goal[0]},{self.goal[1]}"]
        elif self.terminal_select:
            command.append("--select-terminal")
        if self.headless:
            command.append("--headless")
        if self.rectify:
            command.append("--rectify")
        if self.corners:
            command += ["--corners", self.corners]
        if self.arena_width_m is not None:
            command += ["--arena-width-m", str(self.arena_width_m)]
        if self.arena_height_m is not None:
            command += ["--arena-height-m", str(self.arena_height_m)]
        if self.rectify_output_width is not None:
            command += ["--rectify-output-width", str(self.rectify_output_width)]
        if self.grid_size_m is not None:
            command += ["--grid-size-m", str(self.grid_size_m)]
        if self.map_config:
            command += ["--config", os.path.abspath(self.map_config)]
        return command + self._common_args(output_path)

    def _planner_command(self, output_path: str) -> list[str]:
        """构造规划命令，引用当前地图结果并附加算法、随机种子和参数。"""
        command = [
            sys.executable,
            "-m",
            "path_planner.json_pipeline",
            "--map-result",
            self._current_result("map_pipeline"),
        ]
        if self.mode == "research":
            command.append("--research")
            if self.algorithms:
                command += ["--algorithms", ",".join(self.algorithms)]
        elif self.algorithm:
            command += ["--algorithm", self.algorithm]
        if self.seed is not None:
            command += ["--seed", str(self.seed)]
        if self.parameters:
            command += ["--params", json.dumps(self.parameters, ensure_ascii=False)]
        if self.planner_config:
            command += ["--config", os.path.abspath(self.planner_config)]
        return command + self._common_args(output_path)

    def _postprocess_command(self, output_path: str) -> list[str]:
        """构造后处理命令，引用当前地图及规划结果并附加后处理参数。"""
        command = [
            sys.executable,
            "-m",
            "path_postprocess.run_postprocess",
            "--map-result",
            self._current_result("map_pipeline"),
            "--planner-result",
            self._current_result("path_planner"),
        ]
        if self.postprocess_parameters:
            command += [
                "--params",
                json.dumps(self.postprocess_parameters, ensure_ascii=False),
            ]
        if self.postprocess_config:
            command += ["--config", os.path.abspath(self.postprocess_config)]
        return command + self._common_args(output_path)

    def _motion_command(self, output_path: str) -> list[str]:
        """构造运动控制命令，引用后处理结果并传递配置与显式启动确认。"""
        command = [
            sys.executable,
            "-m",
            "motion_control.run_follower",
            "--postprocess-result",
            self._current_result("path_postprocess"),
        ]
        if self.confirm_drive:
            command.append("--yes")
        if self.motion_config:
            command += ["--config", os.path.abspath(self.motion_config)]
        return command + self._common_args(output_path)

    def _current_result(self, module: str) -> str:
        """读取清单中指定模块当前结果路径，作为下游模块的输入。"""
        stage = self.batch.manifest.get("stages", {}).get(module, {})
        result = stage.get("result")
        if not result:
            raise ValueError(f"stage {module} has no result")
        return os.path.abspath(result)


__all__ = ["CommandMixin"]
