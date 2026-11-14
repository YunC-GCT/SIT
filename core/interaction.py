"""Interactive prompts and confirmations for the pipeline orchestrator."""

from __future__ import annotations

import json
import os

import cv2

from .display import display_available, interactive_terminal
from .preview import _destroy_window_safely


class InteractionMixin:
    """Prompt/confirmation methods used by PipelineOrchestrator."""

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
        image = cv2.imread(image_path)
        if image is None:
            return False
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
        if self.recovery is not None:
            return self.recovery(module, errors)
        if self.headless:
            return "exit"
        answer = self.input_fn(
            "Choose retry [r], edit parameters [p], back [b], or exit [q]: "
        ).strip().lower()
        return {"r": "retry", "p": "edit", "b": "back"}.get(answer, "exit")

    def _edit_parameters(self, module: str) -> None:
        if module == "path_planner":
            raw = self.input_fn("New planner parameters as JSON (blank keeps current): ").strip()
            if raw:
                self.parameters = json.loads(raw)
        elif module == "path_postprocess":
            raw = self.input_fn(
                "New postprocess parameters as JSON (blank keeps current): "
            ).strip()
            if raw:
                self.postprocess_parameters = json.loads(raw)
        elif module == "map_pipeline":
            path = self.input_fn(
                "Map config path (blank keeps current): "
            ).strip()
            if path:
                self.map_config = os.path.abspath(path)
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

    def _completion_action(self) -> str:
        answer = self.input_fn(
            "Task complete: rerun [r], change algorithm [a], "
            "reprocess [p], reselect points [s], change map [m], quit [q]: "
        ).strip().lower()
        action = {
            "r": "rerun",
            "a": "algorithm",
            "p": "postprocess",
            "s": "select",
            "m": "map",
            "q": "quit",
        }.get(answer, "quit")
        self.batch.record_event({"type": "completion_action", "action": action})
        return action

    def _apply_completion_action(self, action: str) -> int:
        if action == "algorithm":
            selected = self.input_fn(
                "Algorithm name (or comma-separated names): "
            ).strip()
            names = [name.strip() for name in selected.split(",") if name.strip()]
            if self.mode == "research" or len(names) > 1:
                self.mode = "research"
                self.algorithms = names
                self.algorithm = None
                self.drive = False
            else:
                self.mode = "normal"
                self.algorithm = names[0] if names else None
                self.algorithms = []
                self.drive = not self.headless
            self.batch.set_workflow(mode=self.mode, drive=self.drive)
            return 1
        if action == "postprocess":
            raw = self.input_fn(
                "Postprocess parameters as JSON (blank keeps current): "
            ).strip()
            if raw:
                self.postprocess_parameters = json.loads(raw)
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
            self.image_path = os.path.abspath(
                self.input_fn("Map image path: ").strip()
            )
            self.start = None
            self.goal = None
            self.batch.set_workflow(image=self.image_path, start=None, goal=None)
            return 0
        return 1


__all__ = ["InteractionMixin"]
