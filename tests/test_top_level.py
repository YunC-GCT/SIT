import json
import os
import tempfile
import unittest
from pathlib import Path

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
from core.orchestrator import run_pipeline


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
                os.path.join(tmp, "preview.png"),
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
            self.assertTrue(os.path.exists(report.preview_path))

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


if __name__ == "__main__":
    unittest.main()
