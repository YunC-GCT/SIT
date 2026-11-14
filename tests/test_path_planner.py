import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

from core import (
    STATUS_FAILED,
    STATUS_SUCCESS,
    grid_map_payload,
    make_result,
    read_result,
    write_result,
)
from path_planner import json_pipeline as planner_cli
from path_planner.src.planner_factory import (
    PlannerParameterError,
    create_planner,
    list_algorithms,
    resolve_algorithm_parameters,
    validate_algorithm_parameters,
)
from path_planner.src.planner_pipeline import run_algorithm, run_planning
from path_planner.src.planner_validation import (
    _normalize_path,
    _point,
    _validate_grid,
    _validate_inputs,
)


class PlannerPipelineTests(unittest.TestCase):
    def test_cli_runs_a_planner_and_writes_a_structured_result(self):
        with tempfile.TemporaryDirectory() as tmp:
            map_path = os.path.join(tmp, "map.json")
            write_result(
                map_path,
                make_result(
                    "map_pipeline",
                    "map-cli",
                    STATUS_SUCCESS,
                    outputs={
                        "map": grid_map_payload(
                            np.zeros((6, 6), dtype=np.uint8),
                            grid_size_m=0.3,
                            start=(0, 0),
                            goal=(5, 5),
                        )
                    },
                ),
            )
            output_path = os.path.join(tmp, "planner.json")

            exit_code = planner_cli.main(
                [
                    "--map-result",
                    map_path,
                    "--algorithm",
                    "astar",
                    "--output",
                    output_path,
                    "--run-root",
                    tmp,
                    "--run-id",
                    "planner-cli",
                ]
            )
            result = read_result(output_path)

        self.assertEqual(exit_code, 0)
        self.assertEqual(result["status"], STATUS_SUCCESS)
        self.assertEqual(result["parameters"]["algorithm"], "astar")

    def test_cli_reports_invalid_map_and_conflicting_arguments(self):
        with tempfile.TemporaryDirectory() as tmp:
            missing_map = os.path.join(tmp, "missing.json")
            invalid_output = os.path.join(tmp, "invalid.json")

            exit_code = planner_cli.main(
                [
                    "--map-result",
                    missing_map,
                    "--algorithm",
                    "astar",
                    "--output",
                    invalid_output,
                    "--run-root",
                    tmp,
                    "--run-id",
                    "planner-invalid-map",
                ]
            )
            invalid = read_result(invalid_output)

            map_path = os.path.join(tmp, "map.json")
            write_result(
                map_path,
                make_result(
                    "map_pipeline",
                    "map-cli",
                    STATUS_SUCCESS,
                    outputs={
                        "map": grid_map_payload(
                            np.zeros((6, 6), dtype=np.uint8),
                            grid_size_m=0.3,
                            start=(0, 0),
                            goal=(5, 5),
                        )
                    },
                ),
            )
            conflict_output = os.path.join(tmp, "conflict.json")
            conflict_code = planner_cli.main(
                [
                    "--map-result",
                    map_path,
                    "--algorithm",
                    "astar",
                    "--algorithms",
                    "rrt",
                    "--output",
                    conflict_output,
                    "--run-root",
                    tmp,
                    "--run-id",
                    "planner-conflict",
                ]
            )
            conflict = read_result(conflict_output)

        self.assertEqual(exit_code, 2)
        self.assertEqual(invalid["status"], STATUS_FAILED)
        self.assertEqual(invalid["errors"][0]["code"], "PLANNER_INPUT_ERROR")
        self.assertEqual(conflict_code, 2)
        self.assertEqual(conflict["status"], STATUS_FAILED)

    def test_cli_parameter_helpers_validate_input(self):
        with self.assertRaises(Exception):
            planner_cli._parse_row_col("bad")
        values = planner_cli._parameters_from_cli(
            ["astar"],
            {"algorithm_params": {"astar": {"weight": 1}}},
            '{"astar": {"weight": 2}}',
        )
        self.assertEqual(values["astar"]["weight"], 2)
        global_values = planner_cli._parameters_from_cli(
            ["astar"],
            {},
            '{"weight": 3}',
        )
        self.assertEqual(global_values["astar"]["weight"], 3)
        with self.assertRaises(ValueError):
            planner_cli._parameters_from_cli(
                ["astar"],
                {},
                '{"rrt": {"max_iter": 10}}',
            )

    def test_run_planner_wrapper_exposes_main(self):
        import path_planner.run_planner as wrapper

        self.assertTrue(callable(wrapper.main))

    def test_factory_parameter_validation_and_seed_conflict(self):
        with self.assertRaises(PlannerParameterError):
            validate_algorithm_parameters("missing")
        with self.assertRaises(PlannerParameterError):
            validate_algorithm_parameters("rrt", {"unknown": 1})
        with self.assertRaises(PlannerParameterError):
            validate_algorithm_parameters("rrt", {"max_iter": 0})
        with self.assertRaises(PlannerParameterError):
            validate_algorithm_parameters("rrt", {"step_size": -1})
        with self.assertRaises(PlannerParameterError):
            validate_algorithm_parameters("rrt", {"goal_sample_rate": 2})

        resolved = resolve_algorithm_parameters("rrt", {"max_iter": 10})
        self.assertEqual(resolved["max_iter"], 10)
        self.assertIn("step_size", resolved)

        grid = np.zeros((4, 4), dtype=np.uint8)
        with self.assertRaises(PlannerParameterError):
            create_planner("rrt", grid, params={"seed": 1}, seed=2)
        planner = create_planner("astar", grid)
        self.assertEqual(planner.plan((0, 0), (3, 3))[0], (0, 0))
        self.assertIn("astar", list_algorithms())

    def test_validation_helpers_reject_bad_grid_and_paths(self):
        with self.assertRaises(PlannerParameterError):
            _point((0,), "start")
        with self.assertRaises(PlannerParameterError):
            _point((0.5, 1), "start")
        with self.assertRaises(PlannerParameterError):
            _validate_grid(np.zeros(3))
        with self.assertRaises(PlannerParameterError):
            _validate_grid(np.zeros((0, 3)))

        grid = np.zeros((3, 3), dtype=np.uint8)
        grid[1, 1] = 1
        with self.assertRaises(PlannerParameterError):
            _validate_inputs(grid, (0, 0), (3, 3), 0.3)
        with self.assertRaises(PlannerParameterError):
            _validate_inputs(grid, (0, 0), (1, 1), 0.3)
        with self.assertRaises(PlannerParameterError):
            _validate_inputs(grid, (0, 0), (2, 2), 0)

        self.assertEqual(_normalize_path(None, grid, (0, 0), (2, 2)), [])
        self.assertEqual(
            _normalize_path([(0, 0), (2, 2)], grid, (0, 0), (2, 2)),
            [(0, 0), (2, 2)],
        )
        with self.assertRaises(ValueError):
            _normalize_path([(0, 0)], grid, (0, 0), (2, 2))
        with self.assertRaises(ValueError):
            _normalize_path([(0, 0), (1, 1)], grid, (0, 0), (1, 1))
        with self.assertRaises(ValueError):
            _normalize_path([(0, 0), (3, 3)], grid, (0, 0), (3, 3))

    def test_all_registered_algorithms_share_the_same_result_shape(self):
        grid = np.zeros((12, 12), dtype=np.uint8)
        parameters = {
            "prm": {"num_samples": 120, "k_neighbors": 20, "max_edge_len": 20.0},
            "rrt": {"max_iter": 800, "step_size": 3.0, "goal_sample_rate": 0.3},
            "rrt_star": {
                "max_iter": 800,
                "step_size": 3.0,
                "goal_sample_rate": 0.3,
                "search_radius": 8.0,
            },
        }

        for algorithm in ("astar", "dijkstra", "prm", "rrt", "rrt_star"):
            with self.subTest(algorithm=algorithm):
                result = run_algorithm(
                    grid,
                    (0, 0),
                    (11, 11),
                    algorithm,
                    parameters=parameters.get(algorithm),
                    seed=17,
                    grid_size_m=0.25,
                    run_id="planner-shape",
                )

                self.assertEqual(result["schema_version"], "1.0")
                self.assertEqual(result["module"], "path_planner")
                self.assertEqual(result["status"], STATUS_SUCCESS)
                self.assertEqual(result["outputs"]["raw_path"]["coordinate_system"]["axes"], ["row", "col"])
                points = result["outputs"]["raw_path"]["points"]["values"]
                self.assertEqual(points[0], [0, 0])
                self.assertEqual(points[-1], [11, 11])

                metrics = result["outputs"]["metrics"]["values"]
                self.assertTrue(metrics["found_path"])
                self.assertGreaterEqual(metrics["planning_time_s"], 0.0)
                self.assertGreaterEqual(metrics["raw_node_count"], 2)
                self.assertGreater(metrics["raw_path_length_m"], 0.0)
                self.assertEqual(result["parameters"]["algorithm"], algorithm)
                self.assertEqual(result["parameters"]["seed"], 17)

    def test_random_planner_seed_reproduces_raw_path(self):
        grid = np.zeros((16, 16), dtype=np.uint8)
        parameters = {"max_iter": 1000, "step_size": 3.0, "goal_sample_rate": 0.25}

        first = run_algorithm(
            grid,
            (0, 0),
            (15, 15),
            "rrt",
            parameters=parameters,
            seed=123,
            run_id="reproducible",
        )
        second = run_algorithm(
            grid,
            (0, 0),
            (15, 15),
            "rrt",
            parameters=parameters,
            seed=123,
            run_id="reproducible",
        )

        self.assertEqual(first["status"], STATUS_SUCCESS)
        self.assertEqual(second["status"], STATUS_SUCCESS)
        self.assertEqual(
            first["outputs"]["raw_path"]["points"]["values"],
            second["outputs"]["raw_path"]["points"]["values"],
        )
        self.assertEqual(first["parameters"]["seed"], 123)
        self.assertEqual(first["outputs"]["metrics"]["values"]["random_seed"], 123)

    def test_rrt_star_seed_reproduces_raw_path(self):
        grid = np.zeros((16, 16), dtype=np.uint8)
        parameters = {
            "max_iter": 1000,
            "step_size": 3.0,
            "goal_sample_rate": 0.25,
            "search_radius": 8.0,
        }

        first = run_algorithm(
            grid,
            (0, 0),
            (15, 15),
            "rrt_star",
            parameters=parameters,
            seed=123,
            run_id="reproducible",
        )
        second = run_algorithm(
            grid,
            (0, 0),
            (15, 15),
            "rrt_star",
            parameters=parameters,
            seed=123,
            run_id="reproducible",
        )

        self.assertEqual(first["status"], STATUS_SUCCESS)
        self.assertEqual(second["status"], STATUS_SUCCESS)
        self.assertEqual(
            first["outputs"]["raw_path"]["points"]["values"],
            second["outputs"]["raw_path"]["points"]["values"],
        )
        self.assertEqual(first["outputs"]["metrics"]["values"]["random_seed"], 123)

    def test_no_path_is_a_failed_result_with_structured_error(self):
        grid = np.zeros((7, 7), dtype=np.uint8)
        grid[3, :] = 1
        result = run_algorithm(
            grid,
            (0, 0),
            (6, 6),
            "astar",
            run_id="no-path",
        )

        self.assertEqual(result["status"], STATUS_FAILED)
        self.assertEqual(result["errors"][0]["code"], "NO_PATH")
        self.assertEqual(result["errors"][0]["module"], "path_planner")
        self.assertEqual(result["errors"][0]["stage"], "plan")
        self.assertFalse(result["outputs"]["metrics"]["values"]["found_path"])
        self.assertEqual(result["outputs"]["raw_path"]["points"]["values"], [])

    def test_sampling_budget_failure_does_not_claim_unreachable(self):
        # A connected map, but one RRT expansion cannot reach this goal.
        grid = np.zeros((10, 10), dtype=np.uint8)
        self.assertEqual(run_algorithm(grid, (0, 0), (9, 9), "astar")["status"], STATUS_SUCCESS)
        result = run_algorithm(grid, (0, 0), (9, 9), "rrt",
                               parameters={"max_iter": 1, "step_size": 1.0}, seed=42)
        self.assertEqual(result["errors"][0]["code"], "NO_PATH")
        self.assertEqual(result["errors"][0]["error_type"], "search_budget_exhausted")
        self.assertIn("预算", result["errors"][0]["reason"])

    def test_invalid_parameters_are_a_failed_result_with_structured_error(self):
        result = run_algorithm(
            np.zeros((4, 4), dtype=np.uint8),
            (0, 0),
            (3, 3),
            "rrt",
            parameters={"goal_sample_rate": 2.0},
            run_id="bad-parameters",
        )

        self.assertEqual(result["status"], STATUS_FAILED)
        self.assertEqual(result["errors"][0]["code"], "INVALID_PARAMETERS")
        self.assertEqual(result["errors"][0]["stage"], "validate")
        self.assertIn("goal_sample_rate", result["errors"][0]["reason"])

    def test_research_mode_returns_comparable_results_for_one_run(self):
        result = run_planning(
            np.zeros((8, 8), dtype=np.uint8),
            (0, 0),
            (7, 7),
            ["astar", "dijkstra"],
            parameters={},
            seed=5,
            grid_size_m=0.5,
            run_id="research-001",
            mode="research",
        )

        self.assertEqual(result["status"], STATUS_SUCCESS)
        comparisons = result["outputs"]["comparisons"]
        self.assertEqual([item["algorithm"] for item in comparisons], ["astar", "dijkstra"])
        self.assertTrue(all(item["status"] == STATUS_SUCCESS for item in comparisons))
        self.assertEqual(comparisons[0]["raw_path"]["points"]["shape"][1], 2)
        self.assertEqual(comparisons[0]["metrics"]["units"]["raw_path_length_m"], "m")

    def test_research_mode_is_failed_when_all_selected_algorithms_find_no_path(self):
        grid = np.zeros((6, 6), dtype=np.uint8)
        grid[3, :] = 1
        result = run_planning(
            grid,
            (0, 0),
            (5, 5),
            ["astar", "dijkstra"],
            run_id="research-no-path",
            mode="research",
        )

        self.assertEqual(result["status"], STATUS_FAILED)
        self.assertEqual(len(result["errors"]), 2)
        self.assertTrue(
            all(item["status"] == STATUS_FAILED for item in result["outputs"]["comparisons"])
        )


class PlannerCliTests(unittest.TestCase):
    def test_cli_reads_map_result_and_writes_an_isolated_planner_result(self):
        with tempfile.TemporaryDirectory() as tmp:
            map_result_path = os.path.join(tmp, "map.json")
            map_result = make_result(
                "map_pipeline",
                "map-001",
                STATUS_SUCCESS,
                outputs={
                    "map": grid_map_payload(
                        np.zeros((5, 6), dtype=np.uint8),
                        grid_size_m=0.25,
                        start=(0, 0),
                        goal=(4, 5),
                    )
                },
            )
            write_result(map_result_path, map_result)

            completed = subprocess.run(
                [
                    sys.executable,
                    "-m",
                    "path_planner.run_planner",
                    "--map-result",
                    map_result_path,
                    "--algorithm",
                    "astar",
                    "--run-root",
                    tmp,
                    "--run-id",
                    "planner-001",
                ],
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )

            self.assertEqual(completed.returncode, 0, completed.stderr)
            result_path = Path(tmp) / "planner-001" / "path_planner" / "result.json"
            result = read_result(str(result_path))
            self.assertEqual(result["run_id"], "planner-001")
            self.assertEqual(result["inputs"]["map_result"], os.path.abspath(map_result_path))
            self.assertEqual(result["outputs"]["metrics"]["values"]["found_path"], True)


if __name__ == "__main__":
    unittest.main()
