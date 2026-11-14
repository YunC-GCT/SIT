import io
import json
import os
import subprocess
import sys
import tempfile
import unittest

import main as main_module
from pathlib import Path
from types import SimpleNamespace
from typing import cast
from unittest.mock import mock_open, patch

import cv2
import numpy as np

from core import (
    STATUS_FAILED,
    STATUS_SUCCESS,
    grid_map_payload,
    make_error,
    make_result,
    metric_payload,
    path_payload,
    path_segment_payload,
    save_path_preview,
    write_result,
)
from core.batch import RunBatch
from core.orchestrator import PipelineOrchestrator, run_pipeline


class FakeModuleRunner:
    """A CLI adapter fake that writes the same public JSON contracts as modules."""

    def __init__(self, *, fail_module=None, fail_count=1):
        self.commands = []
        self.fail_module = fail_module
        self.fail_count = fail_count

    def __call__(self, command):
        self.commands.append(list(command))
        module = self._module(command)
        output = self._option(command, "--output")
        run_id = self._option(command, "--run-id")
        if module == "motion_control":
            output = output or ""
        if module == self.fail_module and self.fail_count > 0:
            self.fail_count -= 1
            if output:
                write_result(
                    output,
                    make_result(
                        module,
                        run_id,
                        STATUS_FAILED,
                        errors=[
                            make_error(
                                "FAKE_FAILURE",
                                module=module,
                                stage="execute",
                                error_type="RuntimeError",
                                reason="fake module failure",
                                input_ref="fake-input.json",
                                suggestion="retry the failed stage",
                            )
                        ],
                    ),
                )
            return 2

        if module == "map_pipeline":
            result = make_result(
                module,
                run_id,
                STATUS_SUCCESS,
                outputs={
                    "map": grid_map_payload(
                        [[0, 0, 0], [0, 0, 0], [0, 0, 0]],
                        grid_size_m=0.3,
                        start=(0, 0),
                        goal=(2, 2),
                    )
                },
            )
        elif module == "path_planner":
            result = make_result(
                module,
                run_id,
                STATUS_SUCCESS,
                outputs={
                    "raw_path": path_payload([(0, 0), (0, 1), (1, 1), (2, 2)]),
                    "metrics": metric_payload(
                        {"found_path": True, "planning_time_s": 0.01},
                        {"found_path": "boolean", "planning_time_s": "s"},
                    ),
                },
                parameters={"algorithm": "astar"},
            )
        elif module == "path_postprocess":
            result = make_result(
                module,
                run_id,
                STATUS_SUCCESS,
                outputs={
                    "raw_path": path_payload([(0, 0), (0, 2)]),
                    "smoothed_path": path_payload(
                        [(0.0, 0.0), (0.0, 2.0)],
                        dtype="float64",
                    ),
                    "segments": [
                        path_segment_payload(
                            0,
                            start=(0, 0),
                            end=(0, 2),
                            direction_deg=90.0,
                            turn_angle_deg=90.0,
                            grid_distance=2.0,
                            distance_m=0.6,
                            estimated_time_s=2.0,
                        )
                    ],
                    "smoothing": {"applied": False, "fallback": True},
                    "metrics": metric_payload(
                        {"segment_count": 1, "estimated_execution_time_s": 2.0},
                        {"segment_count": "count", "estimated_execution_time_s": "s"},
                    ),
                },
            )
        elif module == "motion_control":
            result = make_result(
                module,
                run_id,
                STATUS_SUCCESS,
                outputs={"execution": {"completed": True, "stopped": True}},
            )
        else:
            raise AssertionError(f"unexpected command: {command}")

        write_result(output, result)
        return 0

    @staticmethod
    def _module(command):
        for index, value in enumerate(command):
            if value == "-m" and index + 1 < len(command):
                return command[index + 1].split(".")[0]
        raise AssertionError(f"missing module in command: {command}")

    @staticmethod
    def _option(command, name):
        index = command.index(name)
        return command[index + 1]


class TopLevelPipelineTests(unittest.TestCase):
    def test_path_preview_requires_successful_upstream_results(self):
        with tempfile.TemporaryDirectory() as tmp:
            failed_path = os.path.join(tmp, "failed.json")
            write_result(
                failed_path,
                make_result(
                    "map_pipeline",
                    "preview-failed",
                    STATUS_FAILED,
                    errors=[
                        make_error(
                            "MAP_FAILED",
                            module="map_pipeline",
                            stage="process",
                            error_type="ValueError",
                            reason="bad map",
                            input_ref="map.png",
                            suggestion="retry",
                        )
                    ],
                ),
            )
            with self.assertRaises(Exception):
                save_path_preview(
                    failed_path,
                    failed_path,
                    failed_path,
                    os.path.join(tmp, "preview.png"),
                )

    def test_research_path_preview_uses_nested_postprocess_outputs(self):
        with tempfile.TemporaryDirectory() as tmp:
            map_path = os.path.join(tmp, "map.json")
            planner_path = os.path.join(tmp, "planner.json")
            postprocess_path = os.path.join(tmp, "postprocess.json")
            write_result(
                map_path,
                make_result(
                    "map_pipeline",
                    "preview-research",
                    STATUS_SUCCESS,
                    outputs={
                        "map": grid_map_payload(
                            [[0, 0], [0, 0]],
                            grid_size_m=0.3,
                            start=(0, 0),
                            goal=(1, 1),
                        )
                    },
                ),
            )
            write_result(
                planner_path,
                make_result(
                    "path_planner",
                    "preview-research",
                    STATUS_SUCCESS,
                    outputs={
                        "comparisons": [
                            {
                                "algorithm": "astar",
                                "raw_path": path_payload([(0, 0), (1, 1)]),
                                "metrics": metric_payload(
                                    {"planning_time_s": 0.01},
                                    {"planning_time_s": "s"},
                                ),
                            }
                        ]
                    },
                ),
            )
            nested_outputs = {
                "smoothed_path": path_payload(
                    [(0.0, 0.0), (1.0, 1.0)],
                    dtype="float64",
                ),
                "segments": [
                    path_segment_payload(
                        0,
                        start=(0, 0),
                        end=(1, 1),
                        direction_deg=135.0,
                        turn_angle_deg=135.0,
                        grid_distance=2**0.5,
                        distance_m=0.3 * 2**0.5,
                        estimated_time_s=1.0,
                    )
                ],
                "metrics": metric_payload(
                    {"estimated_time_s": 1.0},
                    {"estimated_time_s": "s"},
                ),
            }
            write_result(
                postprocess_path,
                make_result(
                    "path_postprocess",
                    "preview-research",
                    STATUS_SUCCESS,
                    outputs={
                        "comparisons": [
                            {
                                "algorithm": "astar",
                                "status": STATUS_SUCCESS,
                                "outputs": nested_outputs,
                                "errors": [],
                            }
                        ]
                    },
                ),
            )
            preview_path = save_path_preview(
                map_path,
                planner_path,
                postprocess_path,
                os.path.join(tmp, "中文输出", "preview.png"),
            )
            self.assertTrue(os.path.exists(preview_path))

    def test_real_module_clis_share_one_batch_and_produce_headless_preview(self):
        with tempfile.TemporaryDirectory() as tmp:
            image_path = os.path.join(tmp, "map.png")
            config_path = os.path.join(tmp, "map.yaml")
            cv2.imwrite(image_path, np.full((32, 32, 3), 255, dtype=np.uint8))
            Path(config_path).write_text(
                "default_width: 8\n"
                "threshold: 127\n"
                "blur_ksize: 1\n"
                "obstacle_ratio: 0.5\n"
                "grid_size: 0.25\n",
                encoding="utf-8",
            )

            report = run_pipeline(
                image_path,
                start=(0, 0),
                goal=(7, 7),
                mode="normal",
                headless=True,
                run_root=os.path.join(tmp, "runs"),
                run_id="real-cli-001",
                map_config=config_path,
            )

            self.assertEqual(report.status, STATUS_SUCCESS)
            manifest = json.loads(
                Path(tmp, "runs", "real-cli-001", "manifest.json").read_text(
                    encoding="utf-8"
                )
            )
            self.assertEqual(manifest["status"], STATUS_SUCCESS)
            self.assertEqual(
                [
                    manifest["stages"][module]["status"]
                    for module in (
                        "map_pipeline",
                        "path_planner",
                        "path_postprocess",
                    )
                ],
                [STATUS_SUCCESS, STATUS_SUCCESS, STATUS_SUCCESS],
            )
            self.assertEqual(manifest["stages"]["motion_control"]["status"], "pending")
            assert report.preview_path is not None
            self.assertTrue(os.path.exists(report.preview_path))
            for module in ("map_pipeline", "path_planner", "path_postprocess"):
                stage_result = json.loads(
                    Path(manifest["stages"][module]["result"]).read_text(
                        encoding="utf-8"
                    )
                )
                self.assertTrue(stage_result["logs"])
                self.assertTrue(
                    all(os.path.exists(path) for path in stage_result["logs"])
                )

    def test_normal_headless_pipeline_saves_preview_and_skips_motion_by_default(self):
        with tempfile.TemporaryDirectory() as tmp:
            runner = FakeModuleRunner()
            report = run_pipeline(
                "map.png",
                start=(0, 0),
                goal=(2, 2),
                mode="normal",
                headless=True,
                run_root=tmp,
                run_id="normal-001",
                runner=runner,
            )

            self.assertEqual(report.status, STATUS_SUCCESS)
            self.assertFalse(report.motion_started)
            assert report.preview_path is not None
            self.assertTrue(os.path.exists(report.preview_path))
            self.assertNotIn("motion_control", [runner._module(command) for command in runner.commands])
            manifest = json.loads(
                Path(tmp, "normal-001", "manifest.json").read_text(encoding="utf-8")
            )
            self.assertEqual(manifest["workflow"]["mode"], "normal")
            self.assertEqual(manifest["status"], STATUS_SUCCESS)
            self.assertEqual(
                manifest["stages"]["path_postprocess"]["status"],
                STATUS_SUCCESS,
            )

    def test_research_pipeline_passes_algorithms_and_never_starts_motion(self):
        with tempfile.TemporaryDirectory() as tmp:
            runner = FakeModuleRunner()
            report = run_pipeline(
                "map.png",
                start=(0, 0),
                goal=(2, 2),
                mode="research",
                algorithms=["astar", "dijkstra"],
                parameters={"astar": {"seed": 3}},
                seed=11,
                headless=True,
                run_root=tmp,
                run_id="research-001",
                runner=runner,
            )

            self.assertEqual(report.status, STATUS_SUCCESS)
            self.assertFalse(report.motion_started)
            planner_commands = [
                command
                for command in runner.commands
                if runner._module(command) == "path_planner"
            ]
            self.assertEqual(len(planner_commands), 1)
            self.assertIn("--research", planner_commands[0])
            self.assertIn("--algorithms", planner_commands[0])
            self.assertIn("astar,dijkstra", planner_commands[0])

    def test_failed_stage_can_be_retried_without_starting_downstream_motion(self):
        with tempfile.TemporaryDirectory() as tmp:
            runner = FakeModuleRunner(fail_module="path_planner")
            report = run_pipeline(
                "map.png",
                start=(0, 0),
                goal=(2, 2),
                mode="normal",
                headless=True,
                run_root=tmp,
                run_id="retry-001",
                runner=runner,
                recovery=lambda stage, error: "retry",
            )

            self.assertEqual(report.status, STATUS_SUCCESS)
            self.assertEqual(
                [
                    runner._module(command)
                    for command in runner.commands
                ],
                ["map_pipeline", "path_planner", "path_planner", "path_postprocess"],
            )
            manifest = json.loads(
                Path(tmp, "retry-001", "manifest.json").read_text(encoding="utf-8")
            )
            self.assertEqual(manifest["stages"]["path_planner"]["status"], STATUS_SUCCESS)
            self.assertTrue(manifest["stages"]["path_planner"]["attempts"])
            self.assertEqual(
                manifest["stages"]["path_planner"]["attempts"][0]["status"],
                STATUS_FAILED,
            )

    def test_batch_failure_records_structured_error_and_does_not_run_following_modules(self):
        with tempfile.TemporaryDirectory() as tmp:
            runner = FakeModuleRunner(fail_module="path_planner")
            report = run_pipeline(
                "map.png",
                start=(0, 0),
                goal=(2, 2),
                mode="normal",
                headless=True,
                run_root=tmp,
                run_id="failed-001",
                runner=runner,
            )

            self.assertEqual(report.status, STATUS_FAILED)
            self.assertEqual(report.errors[0]["module"], "path_planner")
            self.assertNotIn(
                "path_postprocess",
                [runner._module(command) for command in runner.commands],
            )
            self.assertNotIn(
                "motion_control",
                [runner._module(command) for command in runner.commands],
            )
            manifest = json.loads(
                Path(tmp, "failed-001", "manifest.json").read_text(encoding="utf-8")
            )
            self.assertEqual(manifest["status"], STATUS_FAILED)
            error = manifest["stages"]["path_planner"]["error"][0]
            self.assertEqual(error["stage"], "execute")
            self.assertIn("suggestion", error)

    def test_headless_drive_requires_explicit_path_acceptance(self):
        with tempfile.TemporaryDirectory() as tmp:
            runner = FakeModuleRunner()
            report = run_pipeline(
                "map.png",
                start=(0, 0),
                goal=(2, 2),
                mode="normal",
                headless=True,
                drive=True,
                accept_path=False,
                run_root=tmp,
                run_id="acceptance-001",
                runner=runner,
            )

            self.assertEqual(report.status, STATUS_FAILED)
            self.assertFalse(report.motion_started)
            self.assertEqual(report.errors[0]["code"], "PATH_ACCEPTANCE_REQUIRED")
            self.assertNotIn(
                "motion_control",
                [runner._module(command) for command in runner.commands],
            )

    @patch("core.interaction.cv2")
    def test_closing_path_preview_window_is_a_normal_rejection(self, cv2_mock):
        cv2_mock.imdecode.return_value = np.zeros((8, 8, 3), dtype=np.uint8)
        cv2_mock.waitKey.return_value = -1
        cv2_mock.getWindowProperty.return_value = 0

        orchestrator = PipelineOrchestrator.__new__(PipelineOrchestrator)
        orchestrator.drive = True
        orchestrator.headless = False
        orchestrator.accept_path = False
        orchestrator.batch = cast(RunBatch, SimpleNamespace(
            manifest={"artifacts": {"path_preview": "preview.png"}}
        ))

        with patch("builtins.open", mock_open(read_data=b"encoded-image")):
            accepted = orchestrator._accept_preview()

        self.assertFalse(accepted)
        cv2_mock.destroyWindow.assert_not_called()


    def test_headless_without_points_mentions_terminal_selection(self):
        with tempfile.TemporaryDirectory() as tmp:
            orchestrator = PipelineOrchestrator(
                "map.png",
                headless=True,
                run_root=tmp,
                run_id="headless-001",
            )
            with self.assertRaises(ValueError) as raised:
                orchestrator._validate_inputs()
            self.assertIn("--select-terminal", str(raised.exception))

            orchestrator.terminal_select = True
            orchestrator._validate_inputs()

    def test_map_command_passes_terminal_selection_flag(self):
        with tempfile.TemporaryDirectory() as tmp:
            orchestrator = PipelineOrchestrator(
                "map.png",
                terminal_select=True,
                run_root=tmp,
                run_id="terminal-001",
            )
            command = orchestrator._map_command(os.path.join(tmp, "result.json"))

            self.assertIn("--select-terminal", command)
            self.assertNotIn("--start", command)

    def test_map_command_passes_rectification_flags(self):
        with tempfile.TemporaryDirectory() as tmp:
            orchestrator = PipelineOrchestrator(
                "map.png",
                rectify=True,
                corners="0,0;10,0;10,10;0,10",
                arena_width_m=2.0,
                grid_size_m=0.05,
                run_root=tmp,
                run_id="rect-001",
            )
            command = orchestrator._map_command(os.path.join(tmp, "result.json"))

            self.assertIn("--rectify", command)
            self.assertIn("--corners", command)
            self.assertIn("--arena-width-m", command)
            self.assertIn("--grid-size-m", command)

    @patch("core.interaction.display_available", return_value=False)
    @patch("core.interaction.interactive_terminal", return_value=True)
    def test_terminal_preview_confirmation_accepts_yes(self, _terminal, _display):
        orchestrator = PipelineOrchestrator.__new__(PipelineOrchestrator)
        orchestrator.drive = True
        orchestrator.headless = False
        orchestrator.accept_path = False
        orchestrator.batch = cast(RunBatch, SimpleNamespace(
            manifest={"artifacts": {"path_preview": "preview.png"}}
        ))
        orchestrator.output_fn = lambda _text: None
        orchestrator.input_fn = lambda _prompt: "yes"

        self.assertTrue(orchestrator._accept_preview())

    @patch("core.interaction.display_available", return_value=False)
    @patch("core.interaction.interactive_terminal", return_value=True)
    def test_terminal_preview_confirmation_rejects_other_input(
        self, _terminal, _display
    ):
        orchestrator = PipelineOrchestrator.__new__(PipelineOrchestrator)
        orchestrator.drive = True
        orchestrator.headless = False
        orchestrator.accept_path = False
        orchestrator.batch = cast(RunBatch, SimpleNamespace(
            manifest={"artifacts": {"path_preview": "preview.png"}}
        ))
        orchestrator.output_fn = lambda _text: None
        orchestrator.input_fn = lambda _prompt: "no"

        self.assertFalse(orchestrator._accept_preview())


@patch("core.map_selection.os.path.isfile", return_value=True)
class MapSelectionTests(unittest.TestCase):
    @patch("core.map_selection.os.listdir", return_value=["only.png"])
    @patch("core.map_selection.os.path.isdir", return_value=True)
    def test_single_image_still_prompts_when_interactive(self, _isdir, _listdir, _isfile):
        with patch("builtins.input", return_value="1") as prompt:
            path = main_module._choose_image(None, interactive=True)

        assert path is not None
        self.assertTrue(path.endswith("only.png"))
        prompt.assert_called_once()

    @patch("core.map_selection.os.listdir", return_value=["only.png"])
    @patch("core.map_selection.os.path.isdir", return_value=True)
    def test_blank_choice_uses_the_only_image(self, _isdir, _listdir, _isfile):
        with patch("builtins.input", return_value=""):
            path = main_module._choose_image(None, interactive=True)

        assert path is not None
        self.assertTrue(path.endswith("only.png"))

    @patch("core.map_selection.os.listdir", return_value=["only.png"])
    @patch("core.map_selection.os.path.isdir", return_value=True)
    def test_single_image_does_not_prompt_when_not_interactive(
        self,
        _isdir,
        _listdir,
        _isfile,
    ):
        with patch(
            "builtins.input",
            side_effect=AssertionError("should not prompt"),
        ):
            path = main_module._choose_image(None, interactive=False)

        self.assertIsNone(path)


class CommandBuilderAndProcessRunnerTests(unittest.TestCase):
    def test_research_and_motion_commands_include_expected_flags(self):
        from core.orchestrator import PipelineOrchestrator

        with tempfile.TemporaryDirectory() as tmp:
            orchestrator = PipelineOrchestrator(
                "map.png",
                mode="research",
                algorithms=["astar", "rrt"],
                seed=42,
                parameters={"astar": {"weight": 2}},
                postprocess_parameters={"smooth": False},
                confirm_drive=True,
                run_root=tmp,
                run_id="commands-001",
            )
            orchestrator.batch.update_stage(
                "map_pipeline",
                STATUS_SUCCESS,
                result="map.json",
            )
            orchestrator.batch.update_stage(
                "path_planner",
                STATUS_SUCCESS,
                result="planner.json",
            )
            orchestrator.batch.update_stage(
                "path_postprocess",
                STATUS_SUCCESS,
                result="post.json",
            )

            planner = orchestrator._planner_command("planner-out.json")
            postprocess = orchestrator._postprocess_command("post-out.json")
            motion = orchestrator._motion_command("motion-out.json")

            self.assertIn("--research", planner)
            self.assertIn("--algorithms", planner)
            self.assertIn("astar,rrt", planner)
            self.assertIn("--seed", planner)
            self.assertIn("42", planner)
            self.assertIn("--params", planner)
            self.assertIn("--params", postprocess)
            self.assertIn("--yes", motion)
            self.assertIn("--postprocess-result", motion)
            with self.assertRaises(ValueError):
                orchestrator._current_result("missing_module")

    def test_motion_command_detection_and_stop_only_runner(self):
        from core import process_runner

        self.assertTrue(
            process_runner._is_motion_command(
                [sys.executable, "-m", "motion_control.run_follower"]
            )
        )
        self.assertFalse(
            process_runner._is_motion_command(
                [sys.executable, "-m", "path_planner.json_pipeline"]
            )
        )

        class FakeProcess:
            def __init__(self):
                self.waited = False
                self.killed = False

            def wait(self, timeout=None):
                self.waited = True

            def kill(self):
                self.killed = True

        fake = FakeProcess()
        with patch(
            "core.process_runner.subprocess.Popen",
            return_value=fake,
        ) as popen:
            process_runner._run_stop_only(
                [
                    sys.executable,
                    "-m",
                    "motion_control.run_follower",
                    "--config",
                    "motion.yaml",
                ]
            )

        command = popen.call_args.args[0]
        self.assertIn("--stop-only", command)
        self.assertIn("--config", command)
        self.assertTrue(fake.waited)

    def test_subprocess_runner_returns_exit_code_and_forwards_ctrl_c(self):
        from core import process_runner

        class FakeProcess:
            def wait(self, timeout=None):
                return 7

        with patch(
            "core.process_runner.subprocess.Popen",
            return_value=FakeProcess(),
        ):
            self.assertEqual(
                process_runner._subprocess_runner(
                    [sys.executable, "-m", "path_planner.json_pipeline"]
                ),
                7,
            )

        class InterruptProcess:
            def __init__(self):
                self.calls = 0
                self.signaled = False

            def wait(self, timeout=None):
                self.calls += 1
                if self.calls == 1:
                    raise KeyboardInterrupt
                return 0

            def send_signal(self, _signal):
                self.signaled = True

        process = InterruptProcess()
        with patch(
            "core.process_runner.subprocess.Popen",
            return_value=process,
        ):
            with self.assertRaises(KeyboardInterrupt):
                process_runner._subprocess_runner(
                    [sys.executable, "-m", "motion_control.run_follower"]
                )

        self.assertTrue(process.signaled)

    def test_stop_only_failure_is_reported_to_stderr(self):
        from core import process_runner

        class TimeoutProcess:
            def wait(self, timeout=None):
                raise subprocess.TimeoutExpired(cmd="stop-only", timeout=timeout if timeout is not None else 0)

            def kill(self):
                pass

        stderr = io.StringIO()
        with patch(
            "core.process_runner.subprocess.Popen",
            return_value=TimeoutProcess(),
        ), patch("sys.stderr", stderr):
            process_runner._run_stop_only(
                [sys.executable, "-m", "motion_control.run_follower"]
            )

        self.assertIn("stop-only 失败", stderr.getvalue())


class InteractionMixinTests(unittest.TestCase):
    class FakeBatch:
        def __init__(self):
            self.events = []
            self.workflow = {}

        def record_event(self, event):
            self.events.append(event)

        def set_workflow(self, **kwargs):
            self.workflow.update(kwargs)

    def make_orchestrator(self, answers, *, recovery=None, headless=False):
        orchestrator = PipelineOrchestrator.__new__(PipelineOrchestrator)
        orchestrator.drive = True
        orchestrator.headless = headless
        orchestrator.accept_path = False
        orchestrator.confirm_drive = False
        orchestrator.input_fn = lambda _prompt: next(answers)
        orchestrator.output_fn = lambda _text: None
        orchestrator.recovery = recovery
        orchestrator.batch = cast(RunBatch, self.FakeBatch())
        orchestrator.mode = "normal"
        orchestrator.algorithm = None
        orchestrator.algorithms = []
        orchestrator.parameters = {}
        orchestrator.postprocess_parameters = {}
        orchestrator.map_config = None
        orchestrator.planner_config = None
        orchestrator.postprocess_config = None
        orchestrator.motion_config = None
        orchestrator.start = (0, 0)
        orchestrator.goal = (2, 2)
        orchestrator.image_path = "map.png"
        return orchestrator

    def test_completion_action_maps_keys(self):
        cases = {
            "r": "rerun",
            "a": "algorithm",
            "p": "postprocess",
            "s": "select",
            "m": "map",
            "q": "quit",
        }
        for answer, expected in cases.items():
            with self.subTest(answer=answer):
                orchestrator = self.make_orchestrator(iter([answer]))
                self.assertEqual(orchestrator._completion_action(), expected)
                self.assertEqual(
                    cast(InteractionMixinTests.FakeBatch, orchestrator.batch).events[-1]["type"],
                    "completion_action",
                )

    def test_recovery_action_maps_keys_and_honors_headless_and_callback(self):
        error = {
            "module": "path_planner",
            "stage": "plan",
            "input": "input.json",
            "reason": "boom",
            "suggestion": "retry",
        }
        cases = {
            "r": "retry",
            "p": "edit",
            "b": "back",
            "q": "exit",
        }
        for answer, expected in cases.items():
            with self.subTest(answer=answer):
                orchestrator = self.make_orchestrator(iter([answer]))
                self.assertEqual(
                    orchestrator._recovery_action("path_planner", [error]),
                    expected,
                )

        headless = self.make_orchestrator(iter([]), headless=True)
        self.assertEqual(
            headless._recovery_action("path_planner", [error]),
            "exit",
        )

        callback = self.make_orchestrator(
            iter([]),
            recovery=lambda _module, _errors: "custom",
        )
        self.assertEqual(
            callback._recovery_action("path_planner", [error]),
            "custom",
        )

    def test_apply_completion_action_updates_mode_and_parameters(self):
        single = self.make_orchestrator(iter(["rrt"]))
        self.assertEqual(single._apply_completion_action("algorithm"), 1)
        self.assertEqual(single.mode, "normal")
        self.assertEqual(single.algorithm, "rrt")
        self.assertEqual(cast(InteractionMixinTests.FakeBatch, single.batch).workflow["mode"], "normal")

        research = self.make_orchestrator(iter(["astar,rrt"]))
        self.assertEqual(research._apply_completion_action("algorithm"), 1)
        self.assertEqual(research.mode, "research")
        self.assertEqual(research.algorithms, ["astar", "rrt"])
        self.assertFalse(research.drive)

        postprocess = self.make_orchestrator(iter(['{"smooth": false}']))
        self.assertEqual(postprocess._apply_completion_action("postprocess"), 2)
        self.assertEqual(
            postprocess.postprocess_parameters,
            {"smooth": False},
        )

        select = self.make_orchestrator(iter([]))
        self.assertEqual(select._apply_completion_action("select"), 0)
        self.assertIsNone(select.start)
        self.assertIsNone(select.goal)

        change_map = self.make_orchestrator(iter(["tests/fixtures/01_scattered.png"]))
        self.assertEqual(change_map._apply_completion_action("map"), 0)
        assert change_map.image_path is not None
        self.assertTrue(change_map.image_path.endswith("01_scattered.png"))
        self.assertIsNone(change_map.start)

    def test_edit_parameters_updates_module_specific_values(self):
        planner = self.make_orchestrator(iter(['{"max_iter": 10}']))
        planner._edit_parameters("path_planner")
        self.assertEqual(planner.parameters, {"max_iter": 10})

        postprocess = self.make_orchestrator(iter(['{"smooth": false}']))
        postprocess._edit_parameters("path_postprocess")
        self.assertEqual(postprocess.postprocess_parameters, {"smooth": False})

        map_config = self.make_orchestrator(iter(["tools/acceptance_map.yaml"]))
        map_config._edit_parameters("map_pipeline")
        assert map_config.map_config is not None
        self.assertTrue(map_config.map_config.endswith("acceptance_map.yaml"))

        motion_config = self.make_orchestrator(iter(["motion.yaml"]))
        motion_config._edit_parameters("motion_control")
        assert motion_config.motion_config is not None
        self.assertTrue(motion_config.motion_config.endswith("motion.yaml"))

    def test_confirm_drive_uses_explicit_flag_only_in_headless_mode(self):
        headless = self.make_orchestrator(iter([]), headless=True)
        headless.confirm_drive = True
        self.assertTrue(headless._confirm_drive())
        headless.confirm_drive = False
        self.assertFalse(headless._confirm_drive())

        interactive = self.make_orchestrator(iter([]), headless=False)
        self.assertTrue(interactive._confirm_drive())


if __name__ == "__main__":
    unittest.main()
