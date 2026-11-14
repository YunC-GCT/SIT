"""Top-level workflow orchestration through module CLI contracts."""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Any, Callable, Sequence

from .batch import RunBatch
from .commands import CommandMixin
from .contracts import (
    STATUS_FAILED,
    STATUS_PENDING,
    STATUS_SUCCESS,
    make_error,
    read_result,
)
from .display import display_available, interactive_terminal
from .interaction import InteractionMixin
from .preview import save_path_preview
from .process_runner import _subprocess_runner

MODULE_STAGES = ("map_pipeline", "path_planner", "path_postprocess")


@dataclass
class PipelineReport:
    status: str
    run_id: str
    manifest_path: str
    stage: str | None = None
    preview_path: str | None = None
    motion_started: bool = False
    errors: list[dict] = field(default_factory=list)

    @property
    def exit_code(self) -> int:
        return 0 if self.status == STATUS_SUCCESS else 2


class PipelineOrchestrator(CommandMixin, InteractionMixin):
    """Deep top-level module for normal and research workflow modes."""

    def __init__(
        self,
        image_path: str | None,
        *,
        start: tuple[int, int] | None = None,
        goal: tuple[int, int] | None = None,
        mode: str = "normal",
        algorithm: str | None = None,
        algorithms: Sequence[str] | None = None,
        parameters: dict | None = None,
        postprocess_parameters: dict | None = None,
        seed: int | None = None,
        headless: bool = False,
        terminal_select: bool = False,
        rectify: bool = False,
        corners: str | None = None,
        arena_width_m: float | None = None,
        arena_height_m: float | None = None,
        rectify_output_width: int | None = None,
        grid_size_m: float | None = None,
        drive: bool | None = None,
        accept_path: bool = False,
        confirm_drive: bool = False,
        run_root: str = "runs",
        run_id: str | None = None,
        map_config: str | None = None,
        planner_config: str | None = None,
        postprocess_config: str | None = None,
        motion_config: str | None = None,
        runner: Callable[[Sequence[str]], int] | None = None,
        recovery: Callable[[str, list[dict]], str] | None = None,
        interactive_menu: bool = False,
        input_fn: Callable[[str], str] = input,
        output_fn: Callable[[str], None] = print,
        preview_renderer: Callable[[str, str, str, str], str] = save_path_preview,
    ):
        if mode not in {"normal", "research"}:
            raise ValueError("mode must be normal or research")
        if (start is None) != (goal is None):
            raise ValueError("start and goal must be provided together")
        self.image_path = os.path.abspath(image_path) if image_path else None
        self.start = start
        self.goal = goal
        self.mode = mode
        self.algorithm = algorithm
        self.algorithms = list(algorithms or [])
        self.parameters = dict(parameters or {})
        self.postprocess_parameters = dict(postprocess_parameters or {})
        self.seed = seed
        self.headless = bool(headless)
        self.terminal_select = bool(terminal_select)
        self.rectify = bool(rectify)
        self.corners = corners
        self.arena_width_m = arena_width_m
        self.arena_height_m = arena_height_m
        self.rectify_output_width = rectify_output_width
        self.grid_size_m = grid_size_m
        self.drive = (
            mode == "normal" and not self.headless
            if drive is None
            else bool(drive)
        )
        self.accept_path = bool(accept_path)
        self.confirm_drive = bool(confirm_drive)
        self.map_config = map_config
        self.planner_config = planner_config
        self.postprocess_config = postprocess_config
        self.motion_config = motion_config
        self.runner = runner or _subprocess_runner
        self.recovery = recovery
        self.interactive_menu = bool(interactive_menu)
        self.input_fn = input_fn
        self.output_fn = output_fn
        self.preview_renderer = preview_renderer
        self.batch = RunBatch(root=run_root, run_id=run_id).start()
        self.batch.set_workflow(
            mode=mode,
            headless=self.headless,
            drive=self.drive,
            seed=seed,
            image=self.image_path,
            start=list(start) if start is not None else None,
            goal=list(goal) if goal is not None else None,
            algorithm=algorithm,
            algorithms=self.algorithms,
            parameters=self.parameters,
            postprocess_parameters=self.postprocess_parameters,
        )

    def run(self) -> PipelineReport:
        try:
            self._validate_inputs()
        except ValueError as exc:
            error = make_error(
                "TOP_LEVEL_INPUT_ERROR",
                module="top_level",
                stage="validate",
                error_type=type(exc).__name__,
                reason=str(exc),
                input_ref=self.image_path or self.batch.manifest_path,
                suggestion=(
                    "提供 --start/--goal，或使用 --select-terminal "
                    "在终端输入起终点；否则检查图片路径和配置"
                ),
            )
            return self._failure_report("workflow", error, finish=True)
        try:
            start_index = 0
            while True:
                report = self._run_once(start_index)
                if report.status != STATUS_SUCCESS or not self.interactive_menu:
                    return report
                action = self._completion_action()
                if action == "quit":
                    return report
                start_index = self._apply_completion_action(action)
                self.batch.reload().start()
        except KeyboardInterrupt:
            error = make_error(
                "TOP_LEVEL_INTERRUPTED",
                module="top_level",
                stage="workflow",
                error_type="KeyboardInterrupt",
                reason="workflow interrupted by user",
                input_ref=self.batch.manifest_path,
                suggestion="confirm the vehicle is stopped before rerunning",
            )
            return self._failure_report("workflow", error, finish=True)
        except Exception as exc:
            error = make_error(
                "TOP_LEVEL_ERROR",
                module="top_level",
                stage="validate",
                error_type=type(exc).__name__,
                reason=str(exc),
                input_ref=self.image_path or self.batch.manifest_path,
                suggestion="check workflow arguments and retry the failed batch",
            )
            return self._failure_report("workflow", error, finish=True)

    def _run_once(self, start_index: int) -> PipelineReport:
        while True:
            outcome = self._run_modules(start_index)
            if isinstance(outcome, PipelineReport):
                return outcome
            if outcome is None:
                break
            self.batch.reload().start()
            start_index = outcome

        try:
            preview_path = self._create_preview()
        except Exception as exc:
            return self._failure_report(
                "preview",
                make_error(
                    "PREVIEW_ERROR",
                    module="top_level",
                    stage="preview",
                    error_type=type(exc).__name__,
                    reason=str(exc),
                    input_ref=self._current_result("path_postprocess"),
                    suggestion="inspect the path results and retry preview generation",
                ),
                finish=True,
            )
        if not self._accept_preview():
            return self._failure_report(
                "preview",
                make_error(
                    "PATH_ACCEPTANCE_REQUIRED",
                    module="top_level",
                    stage="preview",
                    error_type="ConfirmationRequired",
                    reason="path preview was not accepted for motion",
                    input_ref=preview_path,
                    suggestion="accept the preview and retry, or run without --drive",
                ),
                finish=True,
            )

        if self.drive:
            if not self._confirm_drive():
                return self._failure_report(
                    "motion_control",
                    make_error(
                        "DRIVE_CONFIRMATION_REQUIRED",
                        module="top_level",
                        stage="motion_control",
                        error_type="ConfirmationRequired",
                        reason="motion was not explicitly confirmed",
                        input_ref=preview_path,
                        suggestion="confirm motion after checking the /cmd_vel preflight",
                    ),
                    finish=True,
                )
            motion_result = self._run_stage_with_recovery(
                "motion_control",
                self._motion_command,
            )
            if motion_result is not None:
                return motion_result
            motion_started = True
        else:
            self.batch.update_stage(
                "motion_control",
                STATUS_PENDING,
                result=None,
            )
            self.batch.set_workflow(motion_skipped_reason="research or drive disabled")
            motion_started = False

        self.batch.record_artifact("path_preview", preview_path)
        self.batch.finish(STATUS_SUCCESS)
        return PipelineReport(
            STATUS_SUCCESS,
            self.batch.run_id,
            self.batch.manifest_path,
            preview_path=preview_path,
            motion_started=motion_started,
        )

    def _validate_inputs(self) -> None:
        if self.image_path is None:
            raise ValueError("an image path is required")
        if self.start is None and not self.terminal_select:
            if self.headless:
                raise ValueError(
                    "无显示器模式需要显式 --start/--goal，"
                    "或使用 --select-terminal 在终端输入起终点"
                )
            if not display_available():
                raise ValueError(
                    "未检测到图形显示：请使用 --headless --start row,col "
                    "--goal row,col，或使用 --select-terminal 在终端输入起终点"
                )
        if self.rectify and not self.corners and not interactive_terminal():
            raise ValueError(
                "透视矫正需要 --corners，或在交互式终端/图形界面中运行"
            )
        if self.start is None:
            if self.terminal_select:
                self.output_fn("start and goal will be entered in the terminal")
            elif not self.headless:
                self.output_fn("start and goal will be selected by the map module")

    def _run_modules(self, start_index: int) -> int | PipelineReport | None:
        for index in range(start_index, len(MODULE_STAGES)):
            module = MODULE_STAGES[index]
            result = self._run_stage_with_recovery(module, self._command_for(module))
            if result is None:
                continue
            if result.stage == "back":
                return max(0, index - 1)
            return result
        return None

    def _run_stage_with_recovery(
        self,
        module: str,
        command_factory: Callable[[str], list[str]],
    ) -> PipelineReport | None:
        while True:
            output_path = self._next_output_path(module)
            result, errors = self._invoke_module(module, command_factory(output_path), output_path)
            if result is not None and result.get("status") == STATUS_SUCCESS:
                return None

            action = self._recovery_action(module, errors)
            self.batch.record_event(
                {
                    "type": "recovery_action",
                    "module": module,
                    "action": action,
                    "errors": errors,
                }
            )
            if action in {"retry", "edit"}:
                if action == "edit":
                    self._edit_parameters(module)
                self.batch.reload().start()
                continue
            if action == "back":
                return PipelineReport(
                    STATUS_FAILED,
                    self.batch.run_id,
                    self.batch.manifest_path,
                    stage="back",
                    errors=errors,
                )
            return self._failure_report(module, errors, finish=True)

    def _invoke_module(
        self,
        module: str,
        command: Sequence[str],
        output_path: str,
    ) -> tuple[dict | None, list[dict]]:
        try:
            return_code = int(self.runner(command))
        except KeyboardInterrupt:
            raise
        except Exception as exc:
            errors = [
                make_error(
                    "MODULE_LAUNCH_ERROR",
                    module=module,
                    stage="launch",
                    error_type=type(exc).__name__,
                    reason=str(exc),
                    input_ref=output_path,
                    suggestion="check the module CLI and retry this stage",
                )
            ]
            self._record_stage_failure(module, output_path, errors)
            return None, errors

        self.batch.reload()
        result = None
        if os.path.exists(output_path):
            try:
                result = read_result(output_path)
                if result.get("module") != module:
                    raise ValueError(
                        f"result module mismatch: expected {module}, got {result.get('module')}"
                    )
                if result.get("run_id") != self.batch.run_id:
                    raise ValueError("result run_id does not match the workflow batch")
            except Exception as exc:
                result = None
                errors = [
                    make_error(
                        "INVALID_MODULE_RESULT",
                        module=module,
                        stage="validate_result",
                        error_type=type(exc).__name__,
                        reason=str(exc),
                        input_ref=output_path,
                        suggestion="inspect the module JSON result and retry",
                    )
                ]
            else:
                if result["status"] == STATUS_SUCCESS and return_code == 0:
                    self._sync_stage(module, output_path, result)
                    return result, []
                errors = result.get("errors") or [
                    make_error(
                        "MODULE_FAILED",
                        module=module,
                        stage="execute",
                        error_type="ModuleExitError",
                        reason=f"module exited with status {return_code}",
                        input_ref=output_path,
                        suggestion="inspect the preserved result and retry",
                    )
                ]
        else:
            errors = [
                make_error(
                    "MODULE_RESULT_MISSING",
                    module=module,
                    stage="write_result",
                    error_type="FileNotFoundError",
                    reason=f"module did not write {output_path}",
                    input_ref=output_path,
                    suggestion="check module logs and retry the failed stage",
                )
            ]

        self._record_stage_failure(module, output_path, errors)
        return result, errors

    def _sync_stage(self, module: str, output_path: str, result: dict) -> None:
        current = self.batch.manifest.get("stages", {}).get(module, {})
        if (
            current.get("status") == STATUS_SUCCESS
            and os.path.abspath(current.get("result", "")) == os.path.abspath(output_path)
        ):
            return
        self.batch.update_stage(module, STATUS_SUCCESS, result=output_path)

    def _record_stage_failure(
        self,
        module: str,
        output_path: str,
        errors: list[dict],
    ) -> None:
        self.batch.reload()
        current = self.batch.manifest.get("stages", {}).get(module, {})
        if (
            current.get("status") == STATUS_FAILED
            and os.path.abspath(current.get("result", "")) == os.path.abspath(output_path)
        ):
            return
        last_attempt = current.get("last_attempt", {})
        if (
            current.get("status") == STATUS_SUCCESS
            and last_attempt.get("status") == STATUS_FAILED
            and os.path.abspath(last_attempt.get("result", "")) == os.path.abspath(output_path)
        ):
            return
        self.batch.update_stage(
            module,
            STATUS_FAILED,
            result=output_path,
            error=errors,
        )
        self.batch.finish(STATUS_FAILED)

    def _failure_report(
        self,
        stage: str,
        errors: dict | list[dict],
        *,
        finish: bool,
    ) -> PipelineReport:
        normalized = errors if isinstance(errors, list) else [errors]
        current = self.batch.manifest.get("stages", {}).get(stage, {})
        if current.get("status") != STATUS_FAILED:
            self.batch.update_stage(
                stage,
                STATUS_FAILED,
                result=self.batch.manifest_path,
                error=normalized,
            )
        if finish:
            self.batch.finish(STATUS_FAILED)
        return PipelineReport(
            STATUS_FAILED,
            self.batch.run_id,
            self.batch.manifest_path,
            stage=stage,
            errors=normalized,
        )

    def _next_output_path(self, module: str) -> str:
        stage = self.batch.manifest.get("stages", {}).get(module)
        base = os.path.join(self.batch.stage_dir(module), "result.json")
        if not stage and not os.path.exists(base):
            return base
        attempts = len((stage or {}).get("attempts", []))
        return os.path.join(
            self.batch.stage_dir(module),
            "attempts",
            f"attempt-{attempts + 2}",
            "result.json",
        )

    def _create_preview(self) -> str:
        map_result = self._current_result("map_pipeline")
        planner_result = self._current_result("path_planner")
        postprocess_result = self._current_result("path_postprocess")
        preview_path = os.path.join(self.batch.path, "preview", "path_preview.png")
        path = self.preview_renderer(
            map_result,
            planner_result,
            postprocess_result,
            preview_path,
        )
        self.batch.record_artifact("path_preview", path)
        return path


def run_pipeline(
    image_path: str | None,
    **kwargs: Any,
) -> PipelineReport:
    """Run the top-level workflow using module CLI contracts."""
    return PipelineOrchestrator(image_path, **kwargs).run()


__all__ = [
    "PipelineOrchestrator",
    "PipelineReport",
    "run_pipeline",
    "save_path_preview",
]
