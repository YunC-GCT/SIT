"""Regression checks through the chooser and complete interactive workflow."""

import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import main
from core.map_selection import choose_map
from core.orchestrator import PipelineOrchestrator
from tests import test_top_level as fixtures

FakeModuleRunner = fixtures.FakeModuleRunner


class MapChooserTests(unittest.TestCase):
    def test_invalid_number_and_path_reprompt_then_accept_number(self):
        with tempfile.TemporaryDirectory() as tmp:
            first = Path(tmp, "a.png")
            second = Path(tmp, "b.png")
            first.touch()
            second.touch()
            Path(tmp, "folder.png").mkdir()
            answers = iter(["0", "-1", "99", "missing.png", "2"])
            output = []
            selected = choose_map(
                directory=tmp, input_fn=lambda _: next(answers), output_fn=output.append,
            )
            self.assertEqual(selected, str(second))
            self.assertEqual(sum("[" in line for line in output), 2)
            self.assertGreaterEqual(sum("请" in line for line in output), 4)

    def test_empty_directory_supports_quoted_custom_path_and_cancel(self):
        with tempfile.TemporaryDirectory() as tmp:
            custom = Path(tmp, "自定义 地图.PNG")
            custom.touch()
            answers = iter(["", f'"{custom}"'])
            self.assertEqual(choose_map(
                directory=str(Path(tmp, "absent")), input_fn=lambda _: next(answers),
                output_fn=lambda _: None,
            ), str(custom))
            self.assertIsNone(choose_map(
                directory=tmp, input_fn=lambda _: "b", output_fn=lambda _: None,
            ))

    def test_blank_preserves_current_or_uses_first_and_rejects_non_image(self):
        with tempfile.TemporaryDirectory() as tmp:
            first, current, config = [Path(tmp, name) for name in ("a.png", "b.png", "x.yaml")]
            for path in (first, current, config):
                path.touch()
            self.assertEqual(choose_map(
                directory=tmp, input_fn=lambda _: "", output_fn=lambda _: None,
            ), str(first))
            answers = iter([str(config), ""])
            self.assertEqual(choose_map(
                directory=tmp, current=str(current), input_fn=lambda _: next(answers),
                output_fn=lambda _: None,
            ), str(current))

    def test_startup_cancellation_does_not_launch_pipeline(self):
        with patch("main.interactive_terminal", return_value=True), patch(
            "main.display_available", return_value=True,
        ), patch("builtins.input", return_value="b"), patch("main.run_pipeline") as run:
            self.assertEqual(main.main(["--no-drive"]), 0)
            run.assert_not_called()


class InteractiveMapWorkflowTests(unittest.TestCase):
    def run_session(self, tmp, answers, runner):
        inputs = iter(answers)
        orchestrator = PipelineOrchestrator(
            "tests/fixtures/01_scattered.png", start=(0, 0), goal=(2, 2),
            drive=False, interactive_menu=True, run_root=tmp, runner=runner,
            input_fn=lambda _: next(inputs), output_fn=lambda _: None,
        )
        with patch("core.orchestrator.display_available", return_value=True):
            report = orchestrator.run()
        self.assertEqual(report.status, "success", report.errors)
        self.assertFalse(report.motion_started)
        return orchestrator

    def test_cancel_change_returns_to_menu_without_rerunning_or_changing_results(self):
        with tempfile.TemporaryDirectory() as tmp:
            runner = FakeModuleRunner()
            orchestrator = self.run_session(tmp, ["m", "b", "q"], runner)
            self.assertEqual(len(runner.commands), 3)
            self.assertEqual(orchestrator.start, (0, 0))
            assert orchestrator.image_path is not None
            self.assertTrue(orchestrator.image_path.endswith("01_scattered.png"))
            manifest = json.loads(Path(orchestrator.batch.manifest_path).read_text(encoding="utf-8"))
            self.assertEqual(manifest["status"], "success")
            self.assertTrue(all(not item.get("attempts") for item in manifest["stages"].values()))

    def test_change_map_by_number_restarts_map_and_clears_points(self):
        with tempfile.TemporaryDirectory() as tmp:
            runner = FakeModuleRunner()
            orchestrator = self.run_session(tmp, ["x", "m", "999", "2", "q"], runner)
            self.assertEqual(len(runner.commands), 6)
            map_command = runner.commands[3]
            self.assertTrue(map_command[3].endswith("maze-maze512-32-0.png"))
            self.assertNotIn("--start", map_command)
            self.assertIsNone(orchestrator.start)
            self.assertIsNone(orchestrator.goal)
            self.assertFalse(orchestrator.drive)

    def test_map_failure_back_or_map_reopens_selection_and_cancel_stays_in_recovery(self):
        for action in ("b", "m"):
            with self.subTest(action=action), tempfile.TemporaryDirectory() as tmp:
                runner = FakeModuleRunner(fail_module="map_pipeline")
                self.run_session(tmp, ["x", action, "b", action, "2", "q"], runner)
                self.assertEqual(len(runner.commands), 4)
                self.assertTrue(runner.commands[1][3].endswith("maze-maze512-32-0.png"))
                self.assertNotIn("--start", runner.commands[1])

    def test_changing_algorithm_keeps_no_drive(self):
        with tempfile.TemporaryDirectory() as tmp:
            runner = FakeModuleRunner()
            self.run_session(tmp, ["a", "dijkstra", "q"], runner)
            self.assertEqual([runner._module(c) for c in runner.commands], [
                "map_pipeline", "path_planner", "path_postprocess",
                "path_planner", "path_postprocess",
            ])

    def test_map_config_invalid_input_does_not_replace_previous_value(self):
        orchestrator = fixtures.InteractionMixinTests().make_orchestrator(iter(["1", "b"]))
        orchestrator.map_config = "original.yaml"
        orchestrator._edit_parameters("map_pipeline")
        self.assertEqual(orchestrator.map_config, "original.yaml")

    def test_map_config_reprompts_until_existing_yaml(self):
        orchestrator = fixtures.InteractionMixinTests().make_orchestrator(iter([
            "1", "tests/fixtures/01_scattered.png", "tools/acceptance_map.yaml",
        ]))
        orchestrator._edit_parameters("map_pipeline")
        self.assertEqual(orchestrator.map_config, os.path.abspath("tools/acceptance_map.yaml"))

    def test_change_map_clears_corners_but_cancel_preserves_them(self):
        orchestrator = fixtures.InteractionMixinTests().make_orchestrator(iter(["b"]))
        orchestrator.corners = "1,1;9,1;9,9;1,9"
        self.assertIsNone(orchestrator._apply_completion_action("map"))
        self.assertEqual(orchestrator.corners, "1,1;9,1;9,9;1,9")
        answers = iter(["tests/fixtures/01_scattered.png"])
        orchestrator.input_fn = lambda _: next(answers)
        self.assertEqual(orchestrator._apply_completion_action("map"), 0)
        self.assertIsNone(orchestrator.corners)


if __name__ == "__main__":
    unittest.main()
