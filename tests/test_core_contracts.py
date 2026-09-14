import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from core import (
    ContractError,
    RunBatch,
    STATUS_FAILED,
    STATUS_PENDING,
    STATUS_SUCCESS,
    make_error,
    make_result,
    read_result,
    require_success,
    write_result,
)
from core.contracts import (
    array_payload,
    grid_map_payload,
    metric_payload,
    module_entry,
    path_payload,
    path_segment_payload,
)


class ContractTests(unittest.TestCase):
    def test_success_result_contains_declared_map_path_segments_metrics_and_modules(self):
        result = make_result(
            "contract_example",
            "run-001",
            STATUS_SUCCESS,
            outputs={
                "map": grid_map_payload(
                    [[0, 1], [0, 0]],
                    grid_size_m=0.3,
                    start=(0, 0),
                    goal=(1, 1),
                ),
                "raw_path": path_payload([(0, 0), (1, 0), (1, 1)]),
                "segments": [
                    path_segment_payload(
                        0,
                        start=(0, 0),
                        end=(1, 0),
                        direction_deg=180.0,
                        turn_angle_deg=180.0,
                        grid_distance=1.0,
                        distance_m=0.3,
                        estimated_time_s=1.0,
                    )
                ],
                "metrics": metric_payload(
                    {"planning_time_s": 0.01, "raw_node_count": 3},
                    {"planning_time_s": "s", "raw_node_count": "count"},
                ),
                "modules": [
                    module_entry("map_pipeline", STATUS_SUCCESS, result_path="map/result.json"),
                    module_entry("path_planner", STATUS_PENDING),
                ],
            },
        )

        self.assertEqual(result["schema_version"], "1.0")
        self.assertEqual(result["logs"], [])
        self.assertEqual(result["outputs"]["map"]["grid"]["shape"], [2, 2])
        self.assertEqual(result["outputs"]["map"]["coordinate_system"]["axes"], ["row", "col"])
        self.assertEqual(result["outputs"]["raw_path"]["points"]["shape"], [3, 2])
        self.assertEqual(result["outputs"]["segments"][0]["units"]["distance_m"], "m")
        self.assertEqual(result["outputs"]["metrics"]["units"]["planning_time_s"], "s")

    def test_failed_result_requires_structured_errors(self):
        with self.assertRaises(ContractError):
            make_result("planner", "run-001", STATUS_FAILED, errors=[])

        error = make_error(
            "NO_PATH",
            module="path_planner",
            stage="plan",
            error_type="unreachable",
            reason="open set exhausted",
            input_ref="runs/run-001/map/result.json",
            suggestion="检查障碍格和起终点",
        )
        result = make_result("path_planner", "run-001", STATUS_FAILED, errors=[error])
        self.assertEqual(result["errors"][0]["module"], "path_planner")
        self.assertEqual(result["errors"][0]["input"], "runs/run-001/map/result.json")

    def test_failure_result_does_not_overwrite_existing_success(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "result.json")
            write_result(path, make_result("module", "run-001", STATUS_SUCCESS))

            failed = make_result(
                "module",
                "run-001",
                STATUS_FAILED,
                errors=[
                    make_error(
                        "BROKEN",
                        module="module",
                        stage="run",
                        error_type="runtime",
                        reason="failed after success",
                        input_ref="input.json",
                        suggestion="retry in a new run directory",
                    )
                ],
            )
            with self.assertRaises(ContractError):
                write_result(path, failed)

            self.assertEqual(read_result(path)["status"], STATUS_SUCCESS)

    def test_downstream_requires_success_schema_and_module(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "result.json")
            write_result(path, make_result("map_pipeline", "run-001", STATUS_SUCCESS))
            self.assertEqual(require_success(path, module="map_pipeline")["module"], "map_pipeline")
            with self.assertRaises(ContractError):
                require_success(path, module="path_planner")

            bad_path = os.path.join(tmp, "bad.json")
            Path(bad_path).write_text(json.dumps({"status": STATUS_SUCCESS}), encoding="utf-8")
            with self.assertRaises(ContractError):
                require_success(bad_path)

    def test_array_payload_records_shape_units_coordinates_and_dtype(self):
        payload = array_payload(
            [[1, 2], [3, 4]],
            dtype="int",
            units={"values": "cell"},
            coordinate_system={"axes": ["row", "col"]},
        )
        self.assertEqual(payload["shape"], [2, 2])
        self.assertEqual(payload["dtype"], "int")
        self.assertEqual(payload["units"]["values"], "cell")

    def test_known_output_payloads_are_validated(self):
        with self.assertRaises(ContractError):
            make_result(
                "map_pipeline",
                "run-001",
                STATUS_SUCCESS,
                outputs={"map": {"grid": [[0]], "grid_size_m": 0.3}},
            )

        with self.assertRaises(ContractError):
            make_result(
                "path_planner",
                "run-001",
                STATUS_SUCCESS,
                outputs={"raw_path": {"points": [[0, 0], [1, 1]]}},
            )

        with self.assertRaises(ContractError):
            make_result(
                "path_planner",
                "run-001",
                STATUS_SUCCESS,
                outputs={"metrics": {"values": {"planning_time_s": 1.0}, "units": {}}},
            )

        bad_path = path_payload([(0, 0), (1, 1)])
        bad_path["points"]["shape"] = [99, 2]
        with self.assertRaises(ContractError):
            make_result(
                "path_planner",
                "run-001",
                STATUS_SUCCESS,
                outputs={"raw_path": bad_path},
            )

        bad_segment = path_segment_payload(
            0,
            start=(0, 0),
            end=(1, 0),
            direction_deg=180.0,
            turn_angle_deg=180.0,
            grid_distance=1.0,
            distance_m=0.3,
            estimated_time_s=1.0,
        )
        bad_segment["delta"] = {"d_row": 0, "d_col": 0}
        with self.assertRaises(ContractError):
            make_result(
                "path_postprocess",
                "run-001",
                STATUS_SUCCESS,
                outputs={"segments": [bad_segment]},
            )


class RunBatchTests(unittest.TestCase):
    def test_run_batch_manifest_tracks_version_time_status_and_refuses_success_regression(self):
        with tempfile.TemporaryDirectory() as tmp:
            batch = RunBatch(root=tmp, run_id="run-001").start()
            result_path = os.path.join(batch.stage_dir("map_pipeline"), "result.json")
            write_result(result_path, make_result("map_pipeline", batch.run_id, STATUS_SUCCESS))
            batch.update_stage("map_pipeline", STATUS_SUCCESS, result=result_path)

            with self.assertRaises(ContractError):
                batch.update_stage("map_pipeline", STATUS_FAILED, error={"reason": "late failure"})

            manifest = json.loads(Path(batch.manifest_path).read_text(encoding="utf-8"))
            self.assertEqual(manifest["schema_version"], "1.0")
            self.assertEqual(manifest["run_id"], "run-001")
            self.assertIn("version", manifest)
            self.assertIn("updated_at", manifest["modules"][0])
            self.assertIn("version", manifest["modules"][0])
            self.assertEqual(manifest["modules"][0]["name"], "map_pipeline")
            self.assertEqual(manifest["stages"]["map_pipeline"]["status"], STATUS_SUCCESS)

    def test_failed_stage_requires_structured_error(self):
        with tempfile.TemporaryDirectory() as tmp:
            batch = RunBatch(root=tmp, run_id="run-001").start()
            with self.assertRaises(ContractError):
                batch.update_stage("path_planner", STATUS_FAILED)
            with self.assertRaises(ContractError):
                batch.update_stage("path_planner", STATUS_FAILED, error=[])
            with self.assertRaises(ContractError):
                batch.update_stage("path_planner", STATUS_FAILED, error={"reason": "missing fields"})


class MinimalContractModuleCliTests(unittest.TestCase):
    def test_minimal_module_cli_writes_success_and_failure_runs_with_logs(self):
        with tempfile.TemporaryDirectory() as tmp:
            script = Path(__file__).resolve().parents[1] / "tools" / "minimal_contract_module.py"
            success = subprocess.run(
                [
                    sys.executable,
                    str(script),
                    "--run-root",
                    tmp,
                    "--run-id",
                    "success-run",
                ],
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )
            self.assertEqual(success.returncode, 0, success.stderr)
            success_result = read_result(os.path.join(tmp, "success-run", "contract_example", "result.json"))
            self.assertEqual(success_result["status"], STATUS_SUCCESS)
            self.assertEqual(len(success_result["logs"]), 1)
            self.assertTrue(os.path.exists(os.path.join(tmp, "success-run", "contract_example", "module.log")))

            failed = subprocess.run(
                [
                    sys.executable,
                    str(script),
                    "--run-root",
                    tmp,
                    "--run-id",
                    "failed-run",
                    "--fail",
                ],
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )
            self.assertEqual(failed.returncode, 2)
            failed_result = read_result(os.path.join(tmp, "failed-run", "contract_example", "result.json"))
            self.assertEqual(failed_result["status"], STATUS_FAILED)
            self.assertEqual(failed_result["errors"][0]["stage"], "example")


if __name__ == "__main__":
    unittest.main()
