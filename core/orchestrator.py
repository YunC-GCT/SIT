"""Top-level workflow orchestration through module CLI contracts."""

from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
from dataclasses import dataclass, field
from typing import Any, Callable, Sequence

import cv2
import numpy as np

from .batch import RunBatch
from .contracts import (
    STATUS_FAILED,
    STATUS_PENDING,
    STATUS_SUCCESS,
    make_error,
    make_result,
    require_success,
    read_result,
)

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
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


def _option(command: Sequence[str], name: str) -> str | None:
    try:
        index = command.index(name)
    except ValueError:
        return None
    if index + 1 >= len(command):
        return None
    return command[index + 1]


def _subprocess_runner(command: Sequence[str]) -> int:
    """Run one module CLI while preserving safe Ctrl+C delivery to children."""
    kwargs: dict[str, Any] = {"cwd": PROJECT_ROOT}
    if sys.platform == "win32":
        kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
    else:
        kwargs["start_new_session"] = True

    process = subprocess.Popen(list(command), **kwargs)
    try:
        return process.wait()
    except KeyboardInterrupt:
        try:
            process.send_signal(
                signal.CTRL_C_EVENT if sys.platform == "win32" else signal.SIGINT
            )
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            if _is_motion_command(command):
                _run_stop_only(command)
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2)
        raise


def _is_motion_command(command: Sequence[str]) -> bool:
    return "-m" in command and "motion_control.run_follower" in command


def _run_stop_only(command: Sequence[str]) -> None:
    stop_command = [
        sys.executable,
        "-m",
        "motion_control.run_follower",
        "--stop-only",
    ]
    if "--config" in command:
        index = command.index("--config")
        if index + 1 < len(command):
            stop_command.extend(["--config", command[index + 1]])
    kwargs: dict[str, Any] = {"cwd": PROJECT_ROOT}
    if sys.platform == "win32":
        kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
    else:
        kwargs["start_new_session"] = True
    try:
        stop_process = subprocess.Popen(stop_command, **kwargs)
        stop_process.wait(timeout=8)
    except (OSError, subprocess.TimeoutExpired):
        # The original process is still terminated below; no successful
        # stop-only command can be claimed when ROS2 is unavailable.
        try:
            stop_process.kill()
        except (UnboundLocalError, OSError):
            pass


def _as_path(points_payload: dict) -> list[tuple[float, float]]:
    values = points_payload.get("points", {}).get("values", [])
    return [(float(point[0]), float(point[1])) for point in values]


def save_path_preview(
    map_result_path: str,
    planner_result_path: str,
    postprocess_result_path: str,
    output_path: str,
) -> str:
    """Render the accepted path view from public JSON result payloads."""
    map_result = require_success(map_result_path, module="map_pipeline")
    planner_result = require_success(planner_result_path, module="path_planner")
    postprocess_result = require_success(
        postprocess_result_path,
        module="path_postprocess",
    )
    grid_payload = map_result["outputs"]["map"]
    grid = np.asarray(grid_payload["grid"]["values"], dtype=np.uint8)
    start_payload = grid_payload["start"]
    goal_payload = grid_payload["goal"]
    start = (int(start_payload["row"]), int(start_payload["col"]))
    goal = (int(goal_payload["row"]), int(goal_payload["col"]))

    from path_planner.preview import draw_comparison, draw_path

    comparisons = planner_result["outputs"].get("comparisons")
    if isinstance(comparisons, list):
        comparison_data = {}
        for item in comparisons:
            raw_path = item.get("raw_path", {})
            comparison_data[item.get("algorithm", "planner")] = {
                "path": [
                    (int(point[0]), int(point[1]))
                    for point in raw_path.get("points", {}).get("values", [])
                ],
                "time": item.get("metrics", {}).get("values", {}).get(
                    "planning_time_s", 0.0
                ),
            }
        image = draw_comparison(grid, comparison_data, start, goal)
    else:
        raw_path = planner_result["outputs"].get("raw_path", {})
        image = draw_path(
            grid,
            [
                (int(point[0]), int(point[1]))
                for point in raw_path.get("points", {}).get("values", [])
            ],
            start,
            goal,
        )

    effective_cell = max(1, image.shape[1] // max(grid.shape[1], 1))
    smoothed = _as_path(postprocess_result["outputs"].get("smoothed_path", {}))
    if len(smoothed) >= 2:
        points = np.asarray(
            [
                [
                    int(round(col * effective_cell + effective_cell / 2)),
                    int(round(row * effective_cell + effective_cell / 2)),
                ]
                for row, col in smoothed
            ],
            dtype=np.int32,
        )
        cv2.polylines(image, [points], False, (0, 128, 255), 2)

    segments = postprocess_result["outputs"].get("segments", [])
    postprocess_comparisons = postprocess_result["outputs"].get("comparisons")
    if not segments and isinstance(postprocess_comparisons, list):
        segments = [
            segment
            for comparison in postprocess_comparisons
            for segment in comparison.get("outputs", {}).get("segments", [])
        ]
    for segment in segments:
        begin = segment["start"]
        end = segment["end"]
        p1 = (
            int(begin["col"] * effective_cell + effective_cell / 2),
            int(begin["row"] * effective_cell + effective_cell / 2),
        )
        p2 = (
            int(end["col"] * effective_cell + effective_cell / 2),
            int(end["row"] * effective_cell + effective_cell / 2),
        )
        cv2.line(image, p1, p2, (0, 165, 0), 1)

    postprocess_outputs = postprocess_result["outputs"]
    if "comparisons" in postprocess_outputs:
        successful = [
            item
            for item in postprocess_outputs["comparisons"]
            if item.get("status") == STATUS_SUCCESS
        ]
        if not successful:
            raise ValueError("postprocess result contains no successful comparison")
        postprocess_outputs = successful[0]["outputs"]
        segments = postprocess_outputs.get("segments", [])
    metrics = postprocess_outputs.get("metrics", {}).get("values", {})
    estimated = metrics.get(
        "estimated_time_s",
        metrics.get("estimated_execution_time_s", 0.0),
    )
    cv2.rectangle(image, (0, 0), (min(image.shape[1], 620), 30), (0, 0, 0), -1)
    cv2.putText(
        image,
        f"segments={len(segments)} estimated={float(estimated):.2f}s open-loop",
        (8, 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    if not cv2.imwrite(output_path, image):
        raise OSError(f"unable to write path preview: {output_path}")
    return os.path.abspath(output_path)


class PipelineOrchestrator:
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
        if self.headless and self.start is None:
            raise ValueError("headless mode requires explicit start and goal")
        if not self.headless and self.start is None:
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

    def _command_for(self, module: str) -> Callable[[str], list[str]]:
        return {
            "map_pipeline": self._map_command,
            "path_planner": self._planner_command,
            "path_postprocess": self._postprocess_command,
        }[module]

    def _common_args(self, output_path: str) -> list[str]:
        return [
            "--output",
            output_path,
            "--run-root",
            self.batch.root,
            "--run-id",
            self.batch.run_id,
        ]

    def _map_command(self, output_path: str) -> list[str]:
        command = [
            sys.executable,
            "-m",
            "map_pipeline.json_pipeline",
            self.image_path or "",
        ]
        if self.start is not None:
            command += ["--start", f"{self.start[0]},{self.start[1]}"]
            command += ["--goal", f"{self.goal[0]},{self.goal[1]}"]
        if self.map_config:
            command += ["--config", os.path.abspath(self.map_config)]
        return command + self._common_args(output_path)

    def _planner_command(self, output_path: str) -> list[str]:
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
        stage = self.batch.manifest.get("stages", {}).get(module, {})
        result = stage.get("result")
        if not result:
            raise ValueError(f"stage {module} has no result")
        return os.path.abspath(result)

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
        image = cv2.imread(image_path)
        if image is None:
            return False
        cv2.namedWindow("Path Preview", cv2.WINDOW_NORMAL)
        cv2.imshow("Path Preview", image)
        key = cv2.waitKey(0) & 0xFF
        cv2.destroyWindow("Path Preview")
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
