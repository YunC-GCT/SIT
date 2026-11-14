import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

from core import (
    grid_map_payload,
    make_result,
    path_payload,
    read_result,
    write_result,
)
from path_postprocess import (
    PathPostprocessParameterError,
    PathSmoother,
    process_path,
    resolve_postprocess_parameters,
)
from path_postprocess.run_postprocess import main as postprocess_main


class CollidingSmoother(PathSmoother):
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
        sampled_points = outputs["smoothed_path"]["points"]["values"]
        self.assertEqual(outputs["smoothed_path"]["points"]["dtype"], "float64")
        self.assertTrue(
            all(
                ((second[0] - first[0]) ** 2 + (second[1] - first[1]) ** 2) ** 0.5
                <= 0.5 + 1e-9
                for first, second in zip(sampled_points, sampled_points[1:])
            )
        )
        self.assertEqual(sampled_points[0], [0.0, 0.0])
        self.assertEqual(sampled_points[-1], [0.0, 6.0])
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

    def test_execution_segments_do_not_depend_on_planner_node_density(self):
        grid = np.zeros((3, 11), dtype=np.uint8)
        dense_path = [(0, col) for col in range(11)]
        sparse_path = [(0, 0), (0, 10)]
        parameters = {"smooth": False, "max_segment_length_grid": 2.0}

        dense_result = process_path(grid, dense_path, parameters=parameters)
        sparse_result = process_path(grid, sparse_path, parameters=parameters)

        self.assertEqual(dense_result["status"], "success")
        self.assertEqual(sparse_result["status"], "success")
        dense_segments = [
            (item["start"], item["end"])
            for item in dense_result["outputs"]["segments"]
        ]
        sparse_segments = [
            (item["start"], item["end"])
            for item in sparse_result["outputs"]["segments"]
        ]
        self.assertEqual(dense_segments, sparse_segments)
        self.assertEqual(len(dense_segments), 5)

    def test_execution_segments_keep_sharp_corners(self):
        grid = np.zeros((4, 4), dtype=np.uint8)

        result = process_path(
            grid,
            [(0, 0), (0, 2), (2, 2)],
            parameters={"smooth": False, "max_segment_length_grid": 4.0},
            run_id="corner-001",
        )

        self.assertEqual(result["status"], "success")
        segments = result["outputs"]["segments"]
        self.assertEqual(len(segments), 2)
        self.assertEqual(segments[0]["start"], {"row": 0, "col": 0})
        self.assertEqual(segments[0]["end"], {"row": 0, "col": 2})
        self.assertEqual(segments[1]["end"], {"row": 2, "col": 2})

    def test_max_segment_length_below_one_grid_is_invalid(self):
        result = process_path(
            np.zeros((3, 3), dtype=np.uint8),
            [(0, 0), (0, 2)],
            parameters={"max_segment_length_grid": 0.5},
        )

        self.assertEqual(result["status"], "failed")
        self.assertEqual(result["errors"][0]["code"], "INVALID_PARAMETERS")

    def test_parameter_aliases_and_validation_errors(self):
        resolved = resolve_postprocess_parameters(
            {
                "sample_spacing_m": 0.06,
                "max_segment_length_m": 0.6,
            },
            grid_size_m=0.3,
        )
        self.assertAlmostEqual(resolved["sample_spacing_grid"], 0.2)
        self.assertAlmostEqual(resolved["max_segment_length_grid"], 2.0)

        with self.assertRaises(PathPostprocessParameterError):
            resolve_postprocess_parameters({"unknown": 1})
        with self.assertRaises(PathPostprocessParameterError):
            resolve_postprocess_parameters({"move_speed_m_s": 0})
        with self.assertRaises(PathPostprocessParameterError):
            resolve_postprocess_parameters({"initial_heading": "up"})
        with self.assertRaises(PathPostprocessParameterError):
            resolve_postprocess_parameters({"max_segment_length_grid": 0.5})

    def test_smoothing_can_be_disabled_and_endpoint_mismatch_fails(self):
        disabled = process_path(
            np.zeros((4, 6), dtype=np.uint8),
            [(0, 0), (0, 5)],
            parameters={"smooth": False, "max_segment_length_grid": 2.0},
        )
        self.assertEqual(disabled["status"], "success")
        self.assertFalse(disabled["outputs"]["smoothing"]["applied"])
        self.assertEqual(disabled["outputs"]["smoothing"]["method"], "none")

        mismatch = process_path(
            np.zeros((4, 6), dtype=np.uint8),
            [(0, 0), (0, 5)],
            start=(0, 1),
            goal=(0, 5),
            parameters={"smooth": False, "max_segment_length_grid": 2.0},
        )
        self.assertEqual(mismatch["status"], "failed")
        self.assertEqual(mismatch["errors"][0]["code"], "INVALID_RAW_PATH")

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

    def test_cli_success_writes_result_with_segments(self):
        with tempfile.TemporaryDirectory() as tmp:
            map_path = os.path.join(tmp, "map.json")
            planner_path = os.path.join(tmp, "planner.json")
            output_path = os.path.join(tmp, "postprocess.json")
            write_result(
                map_path,
                make_result(
                    "map_pipeline",
                    "map-post",
                    "success",
                    outputs={
                        "map": grid_map_payload(
                            np.zeros((4, 6), dtype=np.uint8),
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
                    "planner-post",
                    "success",
                    outputs={"raw_path": path_payload([(0, 0), (0, 5)])},
                ),
            )

            exit_code = postprocess_main(
                [
                    "--map-result",
                    map_path,
                    "--planner-result",
                    planner_path,
                    "--output",
                    output_path,
                    "--run-root",
                    tmp,
                    "--run-id",
                    "post-success",
                ]
            )
            result = read_result(output_path)

        self.assertEqual(exit_code, 0)
        self.assertEqual(result["status"], "success")
        self.assertGreater(len(result["outputs"]["segments"]), 0)

    def test_cli_reports_config_and_map_input_failures(self):
        with tempfile.TemporaryDirectory() as tmp:
            result_path = os.path.join(tmp, "failure.json")
            exit_code = postprocess_main(
                [
                    "--map-result",
                    os.path.join(tmp, "missing-map.json"),
                    "--planner-result",
                    os.path.join(tmp, "missing-planner.json"),
                    "--output",
                    result_path,
                    "--run-root",
                    tmp,
                    "--run-id",
                    "post-failure",
                ]
            )
            result = read_result(result_path)

        self.assertEqual(exit_code, 2)
        self.assertEqual(result["errors"][0]["code"], "POSTPROCESS_MAP_INPUT_ERROR")

        with tempfile.TemporaryDirectory() as tmp:
            result_path = os.path.join(tmp, "config-failure.json")
            exit_code = postprocess_main(
                [
                    "--map-result",
                    os.path.join(tmp, "missing-map.json"),
                    "--planner-result",
                    os.path.join(tmp, "missing-planner.json"),
                    "--params",
                    "not-json",
                    "--output",
                    result_path,
                    "--run-root",
                    tmp,
                    "--run-id",
                    "post-config-failure",
                ]
            )
            result = read_result(result_path)

        self.assertEqual(exit_code, 2)
        self.assertEqual(result["errors"][0]["code"], "POSTPROCESS_CONFIG_ERROR")

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
