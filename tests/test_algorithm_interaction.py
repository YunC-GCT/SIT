"""User-facing algorithm selection and parameter recovery regressions."""

from pathlib import Path
import tempfile
from types import SimpleNamespace
from typing import cast
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from core.orchestrator import PipelineOrchestrator
from core.batch import RunBatch
from core.algorithm_selection import choose_algorithms
from path_planner.catalog import algorithm_catalog
from tests import test_top_level as fixtures


class AlgorithmInteractionTests(unittest.TestCase):
    def test_catalog_tracks_registered_algorithms_and_custom_config(self):
        from path_planner.src.planner_factory import list_algorithms
        catalog, default = algorithm_catalog()
        self.assertEqual([item["name"] for item in catalog], list_algorithms())
        self.assertEqual(default, "astar")
        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp, "planner.yaml")
            config.write_text(
                'path_planner:\n  default_algorithm: dijkstra\n'
                '  available_algorithms: [dijkstra, astar]\n', encoding="utf-8",
            )
            selected = choose_algorithms(
                config_path=str(config), input_fn=lambda _: "1", output_fn=lambda _: None,
            )
            self.assertEqual(selected, ["dijkstra"])

    def test_list_number_reprompts_unknown_name_and_selects_dijkstra(self):
        orchestrator = fixtures.InteractionMixinTests().make_orchestrator(iter(["m", "99", "2"]))
        messages = []
        orchestrator.output_fn = messages.append
        self.assertEqual(orchestrator._apply_completion_action("algorithm"), 1)
        self.assertEqual(orchestrator.algorithm, "dijkstra")
        self.assertTrue(any("[1]" in line and "A*" in line for line in messages))

    def test_cancel_algorithm_keeps_state_without_running_modules(self):
        with tempfile.TemporaryDirectory() as tmp:
            runner = fixtures.FakeModuleRunner()
            answers = iter(["a", "b", "q"])
            orchestrator = PipelineOrchestrator(
                "tests/fixtures/01_scattered.png", start=(0, 0), goal=(2, 2),
                drive=False, interactive_menu=True, run_root=tmp, runner=runner,
                input_fn=lambda _: next(answers), output_fn=lambda _: None,
            )
            with patch("core.orchestrator.display_available", return_value=True):
                report = orchestrator.run()
            self.assertEqual(report.status, "success", report.errors)
            self.assertEqual(len(runner.commands), 3)

    def test_bad_json_reprompts_and_cancel_keeps_parameters(self):
        orchestrator = fixtures.InteractionMixinTests().make_orchestrator(iter(["1", "[]", "{", "b"]))
        orchestrator.parameters = {"max_iter": 100}
        orchestrator._edit_parameters("path_planner")
        self.assertEqual(orchestrator.parameters, {"max_iter": 100})

    def test_multiple_numbers_deduplicate_and_use_research_without_drive(self):
        orchestrator = fixtures.InteractionMixinTests().make_orchestrator(iter(["1，2,1"]))
        self.assertEqual(orchestrator._apply_completion_action("algorithm"), 1)
        self.assertEqual(orchestrator.algorithms, ["astar", "dijkstra"])
        self.assertEqual(orchestrator.mode, "research")
        self.assertFalse(orchestrator.drive)

    def test_blank_algorithm_keeps_temporary_parameters_and_does_not_rerun(self):
        orchestrator = fixtures.InteractionMixinTests().make_orchestrator(iter([""]))
        orchestrator.algorithm = "rrt"
        orchestrator.parameters = {"max_iter": 10}
        self.assertIsNone(orchestrator._apply_completion_action("algorithm"))
        self.assertEqual(orchestrator.algorithm, "rrt")
        self.assertEqual(orchestrator.parameters, {"max_iter": 10})

    def test_new_algorithm_clears_incompatible_temporary_parameters(self):
        orchestrator = fixtures.InteractionMixinTests().make_orchestrator(iter(["2"]))
        orchestrator.algorithm = "rrt"
        orchestrator.parameters = {"max_iter": 10}
        self.assertEqual(orchestrator._apply_completion_action("algorithm"), 1)
        self.assertEqual(orchestrator.parameters, {})

    def test_selecting_effective_default_algorithm_preserves_parameters(self):
        orchestrator = fixtures.InteractionMixinTests().make_orchestrator(iter(["1"]))
        messages = []
        orchestrator.output_fn = messages.append
        orchestrator.parameters = {"astar": {}}
        self.assertIsNone(orchestrator.algorithm)
        self.assertEqual(orchestrator._apply_completion_action("algorithm"), 1)
        self.assertEqual(orchestrator.parameters, {"astar": {}})
        self.assertFalse(any("清除" in message for message in messages))

    def test_invalid_json_can_be_corrected_and_postprocess_cancel_keeps_state(self):
        orchestrator = fixtures.InteractionMixinTests().make_orchestrator(iter([
            "1", "[]", "{", '{"max_iter": 50}',
        ]))
        self.assertTrue(orchestrator._edit_parameters("path_planner"))
        self.assertEqual(orchestrator.parameters, {"max_iter": 50})
        orchestrator.input_fn = lambda _: "b"
        orchestrator.postprocess_parameters = {"smooth": False}
        self.assertIsNone(orchestrator._apply_completion_action("postprocess"))
        self.assertEqual(orchestrator.postprocess_parameters, {"smooth": False})

    def test_failure_selector_and_parameter_cancellations_do_not_retry(self):
        with tempfile.TemporaryDirectory() as tmp:
            runner = fixtures.FakeModuleRunner(fail_module="path_planner")
            answers = iter(["a", "b", "p", "1", "b", "a", "2", "q"])
            orchestrator = PipelineOrchestrator(
                "tests/fixtures/01_scattered.png", start=(0, 0), goal=(2, 2),
                drive=False, interactive_menu=True, run_root=tmp, runner=runner,
                input_fn=lambda _: next(answers), output_fn=lambda _: None,
            )
            with patch("core.orchestrator.display_available", return_value=True):
                report = orchestrator.run()
            self.assertEqual(report.status, "success", report.errors)
            self.assertEqual(len(runner.commands), 4)

    def test_planner_failure_back_selects_algorithm_without_rebuilding_map(self):
        with tempfile.TemporaryDirectory() as tmp:
            runner = fixtures.FakeModuleRunner(fail_module="path_planner")
            answers = iter(["b", "2", "q"])
            orchestrator = PipelineOrchestrator(
                "tests/fixtures/01_scattered.png", start=(0, 0), goal=(2, 2),
                drive=False, interactive_menu=True, run_root=tmp, runner=runner,
                input_fn=lambda _: next(answers), output_fn=lambda _: None,
            )
            with patch("core.orchestrator.display_available", return_value=True):
                report = orchestrator.run()
            self.assertEqual(report.status, "success", report.errors)
            self.assertEqual([runner._module(c) for c in runner.commands], [
                "map_pipeline", "path_planner", "path_planner", "path_postprocess",
            ])
            self.assertEqual(runner._option(runner.commands[2], "--algorithm"), "dijkstra")

    def test_real_chinese_preview_loads_and_reaches_confirmation(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp, "中文路径", "预览.png")
            path.parent.mkdir()
            expected = np.full((9, 12, 3), 127, dtype=np.uint8)
            ok, encoded = cv2.imencode('.png', expected)
            self.assertTrue(ok)
            path.write_bytes(encoded.tobytes())
            orchestrator = fixtures.InteractionMixinTests().make_orchestrator(iter([]))
            orchestrator.batch = cast(RunBatch, SimpleNamespace(
                manifest={"artifacts": {"path_preview": str(path)}},
            ))
            with patch("core.interaction.display_available", return_value=True), patch(
                "core.interaction.cv2.namedWindow",
            ), patch("core.interaction.cv2.imshow") as show, patch(
                "core.interaction.cv2.waitKey", return_value=13,
            ), patch("core.interaction._destroy_window_safely"):
                self.assertTrue(orchestrator._accept_preview())
                np.testing.assert_array_equal(show.call_args.args[1], expected)

    def test_real_module_clis_can_switch_algorithm_in_same_session(self):
        import json
        with tempfile.TemporaryDirectory() as tmp:
            answers = iter(["a", "m", "2", "q"])
            orchestrator = PipelineOrchestrator(
                "tests/fixtures/01_scattered.png", start=(2, 2), goal=(37, 37),
                map_config="tools/acceptance_map.yaml", drive=False,
                interactive_menu=True, run_root=tmp,
                input_fn=lambda _: next(answers), output_fn=lambda _: None,
            )
            with patch("core.orchestrator.display_available", return_value=True):
                report = orchestrator.run()
            self.assertEqual(report.status, "success", report.errors)
            manifest = json.loads(Path(report.manifest_path).read_text(encoding="utf-8"))
            planner = json.loads(Path(
                manifest["stages"]["path_planner"]["result"],
            ).read_text(encoding="utf-8"))
            self.assertEqual(planner["parameters"]["algorithm"], "dijkstra")
            self.assertNotIn("attempts", manifest["stages"]["map_pipeline"])
            self.assertEqual(manifest["stages"]["motion_control"]["status"], "pending")


if __name__ == "__main__":
    unittest.main()
