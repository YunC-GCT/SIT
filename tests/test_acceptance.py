import json
import os
import tempfile
import unittest
from pathlib import Path

from core.acceptance import run_simulated_acceptance
from motion_control.src.simulation import (
    simulate_exception_stop,
    simulate_keyboard_interrupt_stop,
    simulate_segments,
)


class AcceptanceTests(unittest.TestCase):
    def test_fake_ros2_executes_turn_zero_forward_and_safe_stop(self):
        simulation = simulate_segments(
            [
                {"index": 0, "direction_deg": 0.0, "distance_m": 0.3},
                {"index": 1, "direction_deg": 90.0, "distance_m": 0.3},
                {"index": 2, "direction_deg": 180.0, "distance_m": 0.3},
                {"index": 3, "direction_deg": 270.0, "distance_m": 0.3},
            ],
            turn_speed=1.0,
            move_speed=0.5,
            timer_period=0.1,
            stop_publish_count=2,
            stop_publish_period_s=0.0,
            communication_buffer_s=0.0,
        )

        self.assertTrue(simulation.preflight.confirmed)
        self.assertEqual(simulation.preflight.topic, "/cmd_vel")
        self.assertEqual(simulation.preflight.subscribers, ("/mecanum_chassis_node",))
        self.assertEqual(simulation.preflight.other_publishers, ())
        self.assertTrue(simulation.report.completed)
        self.assertTrue(simulation.report.stopped)
        self.assertTrue(simulation.node_destroyed)
        self.assertEqual(simulation.commands[-1], (0.0, 0.0, 0.0))
        self.assertIn("destroy_node", simulation.events)

        moving = [
            command
            for command in simulation.commands
            if command != (0.0, 0.0, 0.0)
        ]
        self.assertTrue(any(command[0] > 0 for command in moving))
        self.assertTrue(any(command[2] != 0 for command in moving))
        self.assertTrue(all(command[1] == 0 for command in moving))

    def test_fake_ros2_stops_on_keyboard_interrupt_and_exception(self):
        segments = [{"index": 0, "direction_deg": 90.0, "distance_m": 0.3}]

        interrupted = simulate_keyboard_interrupt_stop(segments)
        self.assertEqual(interrupted.exception_type, "KeyboardInterrupt")
        self.assertTrue(interrupted.stopped)
        self.assertTrue(interrupted.node_destroyed)
        self.assertTrue(interrupted.zero_before_destroy)

        failed = simulate_exception_stop(segments)
        self.assertEqual(failed.exception_type, "RuntimeError")
        self.assertTrue(failed.stopped)
        self.assertTrue(failed.node_destroyed)
        self.assertTrue(failed.zero_before_destroy)

    def test_fixed_pipeline_acceptance_preserves_stage_logs_and_manual_record(self):
        project_root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as tmp:
            result = run_simulated_acceptance(
                start=(2, 2),
                goal=(37, 37),
                seed=42,
                run_root=tmp,
                run_id="acceptance-001",
                map_config=str(project_root / "tools" / "acceptance_map.yaml"),
            )

            self.assertEqual(result["status"], "success")
            run_path = Path(tmp, "acceptance-001")
            manifest = json.loads(
                (run_path / "manifest.json").read_text(encoding="utf-8")
            )
            self.assertEqual(manifest["status"], "success")
            self.assertEqual(manifest["workflow"]["seed"], 42)
            for module in ("map_pipeline", "path_planner", "path_postprocess"):
                self.assertEqual(manifest["stages"][module]["status"], "success")
                stage_result = json.loads(
                    Path(manifest["stages"][module]["result"]).read_text(
                        encoding="utf-8"
                    )
                )
                self.assertTrue(stage_result["logs"])
                self.assertTrue(
                    all(os.path.exists(path) for path in stage_result["logs"])
                )

            self.assertEqual(manifest["stages"]["acceptance"]["status"], "success")
            self.assertTrue(
                (run_path / "acceptance" / "manual_record.json").exists()
            )
            simulation = result["outputs"]["simulation"]["execution"]
            self.assertTrue(simulation["completed"])
            self.assertTrue(simulation["stopped"])
            self.assertEqual(
                result["outputs"]["manual_hardware_acceptance"]["status"],
                "pending_manual",
            )

    def test_research_acceptance_uses_comparison_mode_without_motion_stage(self):
        project_root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as tmp:
            result = run_simulated_acceptance(
                start=(2, 2),
                goal=(37, 37),
                mode="research",
                algorithms=["astar", "dijkstra"],
                seed=42,
                run_root=tmp,
                run_id="acceptance-research-001",
                map_config=str(project_root / "tools" / "acceptance_map.yaml"),
            )

            self.assertEqual(result["status"], "success")
            manifest = json.loads(
                Path(tmp, "acceptance-research-001", "manifest.json").read_text(
                    encoding="utf-8"
                )
            )
            self.assertEqual(manifest["workflow"]["mode"], "research")
            self.assertEqual(manifest["stages"]["motion_control"]["status"], "pending")
            planner = json.loads(
                Path(manifest["stages"]["path_planner"]["result"]).read_text(
                    encoding="utf-8"
                )
            )
            self.assertIn("comparisons", planner["outputs"])


if __name__ == "__main__":
    unittest.main()
