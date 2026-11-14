"""Interactive prompts and confirmations for the pipeline orchestrator."""

from __future__ import annotations

import json
import os
from typing import Callable
from .batch import RunBatch

import cv2
import numpy as np

from .display import display_available, interactive_terminal
from .preview import _destroy_window_safely
from .map_selection import choose_map
from .algorithm_selection import choose_algorithms
from path_planner.catalog import algorithm_catalog


class InteractionMixin:
    """Prompt/confirmation methods used by PipelineOrchestrator."""

    batch: RunBatch
    drive: bool
    headless: bool
    accept_path: bool
    confirm_drive: bool
    recovery: Callable[[str, list[dict]], str] | None
    input_fn: Callable[[str], str]
    output_fn: Callable[[str], None]
    planner_config: str | None
    postprocess_config: str | None

    def _accept_preview(self) -> bool:
        if not self.drive:
            return True
        if self.headless:
            return self.accept_path
        if self.accept_path:
            return True
        image_path = self.batch.manifest.get("artifacts", {}).get("path_preview")
        if not image_path:
            return False
        if not display_available():
            if not interactive_terminal():
                return False
            self.output_fn(f"路径预览已保存: {image_path}")
            answer = self.input_fn(
                "确认接受该路径？输入 yes 继续: "
            ).strip().lower()
            return answer == "yes"
        # OpenCV imread cannot reliably open Unicode Windows paths.
        with open(image_path, "rb") as handle:
            image = cv2.imdecode(np.frombuffer(handle.read(), dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError(f"无法解码路径预览图片：{image_path}")
        cv2.namedWindow("Path Preview", cv2.WINDOW_NORMAL)
        cv2.imshow("Path Preview", image)
        key = -1
        try:
            key = cv2.waitKey(0) & 0xFF
        except cv2.error:
            pass
        finally:
            _destroy_window_safely("Path Preview")
        return key in {10, 13, 32, ord("y"), ord("Y")}

    def _confirm_drive(self) -> bool:
        if self.headless:
            return self.confirm_drive
        # The motion-control CLI performs the /cmd_vel preflight and asks for
        # the final interactive confirmation after printing its graph report.
        return True

    def _recovery_action(self, module: str, errors: list[dict]) -> str:
        for error in errors:
            self.output_fn(
                f"[top-level] {error.get('module', module)} "
                f"{error.get('stage', 'unknown')}: "
                f"input={error.get('input', '<unknown>')} "
                f"{error.get('reason', 'unknown error')} "
                f"Suggestion: {error.get('suggestion', 'retry')}"
            )
        if module == "map_pipeline":
            self.output_fn("地图输入可用 m / b 重新选择；栅格配置请用 p 提供 YAML 文件路径。")
        if self.recovery is not None:
            return self.recovery(module, errors)
        if self.headless:
            return "exit"
        choices = {"r": "retry", "p": "edit", "b": "back", "q": "exit"}
        prompt = "失败恢复：重试 [r]，修改参数 [p]，返回上一步 [b]，退出 [q]："
        if module == "map_pipeline":
            choices["m"] = "map"
            prompt = (
                "地图处理失败：重试 [r]，重新选地图 [m]，修改栅格配置 [p]，"
                "返回地图选择 [b]，退出 [q]："
            )
        elif module == "path_planner":
            choices["a"] = "algorithm"
            prompt = "规划失败：重试 [r]，重选算法 [a]，修改参数 [p]，返回算法选择 [b]，退出 [q]："
        while True:
            answer = self.input_fn(prompt).strip().lower()
            if answer in choices:
                return choices[answer]
            self.output_fn("无效选项，请输入括号中的字母。")

    def _prompt_parameters(self, current: dict, label: str) -> dict | None:
        self.output_fn(f"当前{label}参数：{json.dumps(current, ensure_ascii=False)}")
        while True:
            raw = self.input_fn(
                f"输入{label}参数 JSON 对象（例如 {{}}；回车保留，b 取消）："
            ).strip()
            if raw.lower() == "b":
                return None
            if not raw:
                return current
            try:
                values = json.loads(raw)
                if not isinstance(values, dict):
                    raise ValueError("必须输入 JSON 对象，例如 {}；不能输入数字、数组或字符串")
            except ValueError as exc:
                self.output_fn(f"参数无效：{exc}；请重新输入。")
                continue
            return values

    def _edit_parameters(self, module: str) -> bool:
        if module == "path_planner":
            values = self._prompt_parameters(self.parameters, "规划")
            if values is None:
                return False
            self.parameters = values
        elif module == "path_postprocess":
            values = self._prompt_parameters(self.postprocess_parameters, "后处理")
            if values is None:
                return False
            self.postprocess_parameters = values
        elif module == "map_pipeline":
            while True:
                path = self.input_fn(
                    "栅格配置 YAML 文件路径（不是地图编号；回车保留，b 取消）："
                ).strip().strip('"').strip("'")
                if not path or path.lower() == "b":
                    return False
                path = os.path.abspath(os.path.expanduser(path))
                if not os.path.isfile(path) or not path.lower().endswith((".yaml", ".yml")):
                    self.output_fn("配置文件不存在或不是 YAML 文件，请重新输入。")
                    continue
                self.map_config = path
                break
        elif module == "motion_control":
            path = self.input_fn(
                "Motion config path (blank keeps current): "
            ).strip()
            if path:
                self.motion_config = os.path.abspath(path)
        self.batch.set_workflow(
            map_config=self.map_config,
            planner_config=self.planner_config,
            postprocess_config=self.postprocess_config,
            motion_config=self.motion_config,
            parameters=self.parameters,
            postprocess_parameters=self.postprocess_parameters,
        )
        return True

    def _completion_action(self) -> str:
        choices = {
            "r": "rerun",
            "a": "algorithm",
            "p": "postprocess",
            "s": "select",
            "m": "map",
            "q": "quit",
        }
        while True:
            answer = self.input_fn(
                "任务完成：重跑 [r]，换算法 [a]，调整后处理 [p]，"
                "重选起终点 [s]，换地图 [m]，退出 [q]："
            ).strip().lower()
            if answer in choices:
                action = choices[answer]
                break
            self.output_fn("无效选项，请输入括号中的字母。")
        self.batch.record_event({"type": "completion_action", "action": action})
        return action

    def _apply_completion_action(self, action: str) -> int | None:
        if action == "algorithm":
            return 1 if self._select_algorithms() else None
        if action == "postprocess":
            values = self._prompt_parameters(self.postprocess_parameters, "后处理")
            if values is None:
                return None
            self.postprocess_parameters = values
            self.batch.set_workflow(
                postprocess_parameters=self.postprocess_parameters,
            )
            return 2
        if action == "select":
            self.start = None
            self.goal = None
            self.batch.set_workflow(start=None, goal=None)
            return 0
        if action == "map":
            return 0 if self._select_map() else None
        return 1

    def _select_algorithms(self) -> bool:
        _, default = algorithm_catalog(self.planner_config)
        previous = self.algorithms if self.mode == "research" else [self.algorithm or default]
        names = choose_algorithms(
            current=previous, config_path=self.planner_config,
            input_fn=self.input_fn, output_fn=self.output_fn,
        )
        if names is None:
            self.output_fn("已取消换算法，保留当前算法、参数和已有结果。")
            return False
        if names != previous:
            self.parameters = {}
            self.output_fn("切换算法后使用配置中的默认参数，已清除上一算法的临时参数。")
        if self.mode == "research" or len(names) > 1:
            self.mode, self.algorithms, self.algorithm = "research", names, None
            self.drive = False
        else:
            self.mode, self.algorithms, self.algorithm = "normal", [], names[0]
        self.batch.set_workflow(
            mode=self.mode, drive=self.drive, algorithm=self.algorithm,
            algorithms=self.algorithms, parameters=self.parameters,
        )
        return True

    def _select_map(self) -> bool:
        path = choose_map(
            current=self.image_path, input_fn=self.input_fn, output_fn=self.output_fn,
        )
        if path is None:
            self.output_fn("已取消换图，保留当前地图和已有结果。")
            return False
        if path != self.image_path:
            # Corners are pixel coordinates belonging to the previous image.
            self.corners = None
        self.image_path = path
        self.start = None
        self.goal = None
        self.batch.set_workflow(image=path, start=None, goal=None)
        self.output_fn("接下来请重新选择起点和终点。")
        return True


__all__ = ["InteractionMixin"]
