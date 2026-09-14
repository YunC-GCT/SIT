import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

from core import grid_map_payload, make_result, path_payload, read_result, write_result
from path_postprocess import process_path


class CollidingSmoother:
    def smooth(self, path, sample_spacing_grid):
        return [(0.0, 0.0), (0.5, 0.5), (2.0, 2.0)]


class PathPostprocessTests(unittest.TestCase):
    def test_smoothing_preserves_endpoints_and_returns_executable_path(self):
        result = process_path(
            np.zeros((8, 8), dtype=np.uint8),
            [(0, 0), (0, 3), (0, 6)],
            grid_size_m=0.3,
            parameters={
                "sample_spacing_grid": 0.5,
                "max_segment_length_grid": 2.0,
            },
            run_id="smooth-001",
        )

        self.assertEqual(result["status"], "success")
        outputs = result["outputs"]
        self.assertTrue(outputs["smoothing"]["applied"])
        self.assertFalse(outputs["smoothing"]["fallback"])
        self.assertEqual(outputs["smoothed_path"]["points"]["values"][0], [0, 0])
        self.assertEqual(outputs["smoothed_path"]["points"]["values"][-1], [0, 6])
        self.assertEqual(outputs["segments"][0]["start"], {"row": 0, "col": 0})
        self.assertEqual(outputs["segments"][-1]["end"], {"row": 0, "col": 6})

    def test_collision_in_smoothed_curve_falls_back_to_validated_raw_path(self):
        grid = np.zeros((4, 4), dtype=np.uint8)
        grid[1, 1] = 1

        result = process_path(
            grid,
            [(0, 0), (0, 2), (2, 2)],
            parameters={"max_segment_length_grid": 2.0},
            smoother=CollidingSmoother(),
            run_id="fallback-001",
        )

        self.assertEqual(result["status"], "success")
        smoothing = result["outputs"]["smoothing"]
        self.assertFalse(smoothing["applied"])
        self.assertTrue(smoothing["fallback"])
        self.assertEqual(smoothing["fallback_reason"], "collision")
        self.assertEqual(
            result["outputs"]["smoothed_path"]["points"]["values"],
            [[0, 0], [0, 2], [2, 2]],
        )

    def test_max_segment_length_splits_arbitrary_axis_step_and_calculates_angles(self):
        result = process_path(
            np.zeros((3, 8), dtype=np.uint8),
            [(0, 0), (0, 5)],
            parameters={
                "max_segment_length_grid": 2.0,
                "move_speed_m_s": 0.5,
                "initial_heading": "north",
            },
            run_id="segments-001",
        )

        self.assertEqual(result["status"], "success")
        segments = result["outputs"]["segments"]
        self.assertTrue(result["outputs"]["smoothing"]["fallback"])
        self.assertEqual(
            result["outputs"]["smoothing"]["fallback_reason"],
            "insufficient_points",
        )
        self.assertEqual(len(segments), 3)
        self.assertEqual(
            [(item["start"], item["end"]) for item in segments],
            [
                ({"row": 0, "col": 0}, {"row": 0, "col": 2}),
                ({"row": 0, "col": 2}, {"row": 0, "col": 4}),
                ({"row": 0, "col": 4}, {"row": 0, "col": 5}),
            ],
        )
        self.assertEqual(segments[0]["direction_deg"], 90.0)
        self.assertEqual(segments[0]["turn_angle_deg"], 90.0)
        self.assertEqual(segments[1]["turn_angle_deg"], 0.0)
        self.assertEqual(segments[0]["distance_m"], 0.6)
        self.assertGreater(segments[0]["estimated_time_s"], 0.0)

    def test_invalid_raw_path_is_a_structured_failure(self):
        grid = np.zeros((3, 3), dtype=np.uint8)
        grid[1, 1] = 1

        result = process_path(
            grid,
            [(0, 0), (2, 2)],
            run_id="invalid-raw-001",
        )

        self.assertEqual(result["status"], "failed")
        self.assertEqual(result["errors"][0]["code"], "INVALID_RAW_PATH")
        self.assertEqual(result["errors"][0]["module"], "path_postprocess")
        self.assertEqual(result["errors"][0]["stage"], "validate_raw_path")

    def test_cli_reads_upstream_results_and_writes_an_isolated_result(self):
        with tempfile.TemporaryDirectory() as tmp:
            map_path = os.path.join(tmp, "map.json")
            planner_path = os.path.join(tmp, "planner.json")
            write_result(
                map_path,
                make_result(
                    "map_pipeline",
                    "map-001",
                    "success",
                    outputs={
                        "map": grid_map_payload(
                            np.zeros((3, 6), dtype=np.uint8),
                            grid_size_m=0.3,
                            start=(0, 0),
                            goal=(0, 5),
                        )
                    },
                ),
            )
            write_result(
                planner_path,
                make_result(
                    "path_planner",
                    "planner-001",
                    "success",
                    outputs={"raw_path": path_payload([(0, 0), (0, 5)])},
                ),
            )

            completed = subprocess.run(
                [
                    sys.executable,
                    "-m",
                    "path_postprocess.run_postprocess",
                    "--map-result",
                    map_path,
                    "--planner-result",
                    planner_path,
                    "--run-root",
                    tmp,
                    "--run-id",
                    "postprocess-001",
                ],
                cwd=Path(__file__).resolve().parents[1],
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )

            self.assertEqual(completed.returncode, 0, completed.stderr)
            result_path = (
                Path(tmp)
                / "postprocess-001"
                / "path_postprocess"
                / "result.json"
            )
            result = read_result(str(result_path))
            self.assertEqual(result["module"], "path_postprocess")
            self.assertEqual(result["status"], "success")
            self.assertEqual(
                result["outputs"]["segments"][-1]["end"],
                {"row": 0, "col": 5},
            )


if __name__ == "__main__":
    unittest.main()
