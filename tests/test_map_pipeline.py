import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np

from core import STATUS_FAILED, STATUS_SUCCESS, read_result
from map_pipeline.json_pipeline import process
from map_pipeline.src.coordinate_mapper import (
    map_pixel_to_grid,
    validate_grid_points,
)
from map_pipeline.src.grid_map import GridMap
from map_pipeline.src.interactive_picker import PointSelectionCancelled, pick_points
from map_pipeline.src.map_loader import create_grid_map


class MapPipelineTests(unittest.TestCase):
    def test_pixel_coordinates_map_to_row_col_and_clip_only_at_grid_boundary(self):
        grid_map = GridMap(
            np.zeros((4, 5), dtype=np.uint8),
            cell_width=10,
            cell_height=8,
            img_shape=(32, 50),
        )

        self.assertEqual(map_pixel_to_grid(19, 15, grid_map), (1, 1))
        self.assertEqual(map_pixel_to_grid(999, 999, grid_map), (3, 4))

    def test_start_and_goal_on_obstacles_are_rejected_without_relocation(self):
        grid_map = GridMap(
            np.array([[0, 1], [0, 0]], dtype=np.uint8),
            cell_width=10,
            cell_height=10,
            img_shape=(20, 20),
        )

        with self.assertRaisesRegex(ValueError, "start.*障碍"):
            validate_grid_points(grid_map, (0, 1), (1, 1))

        self.assertEqual(
            validate_grid_points(grid_map, (0, 0), (1, 1)),
            ((0, 0), (1, 1)),
        )

    def test_grid_width_larger_than_image_is_rejected_before_zero_pixel_cells(self):
        with self.assertRaisesRegex(ValueError, "不能大于图片宽度"):
            create_grid_map(np.zeros((4, 4), dtype=np.uint8), target_width=5)

    def test_json_process_rejects_obstacle_and_does_not_write_preview(self):
        with tempfile.TemporaryDirectory() as tmp:
            image_path = self._write_test_image(tmp)
            output_path = os.path.join(tmp, "map_result.json")

            result = process(
                image_path,
                start=(0, 0),
                goal=(3, 3),
                config=self._config(),
                output_path=output_path,
            )

            self.assertEqual(result["status"], STATUS_FAILED)
            self.assertIn("障碍", result["errors"][0]["reason"])
            self.assertFalse(os.path.exists(os.path.splitext(output_path)[0] + "_preview.png"))
            self.assertEqual(read_result(output_path)["status"], STATUS_FAILED)

    def test_headless_cli_accepts_grid_points_and_saves_versioned_result_and_preview(self):
        with tempfile.TemporaryDirectory() as tmp:
            image_path = self._write_test_image(tmp)
            output_path = os.path.join(tmp, "map_result.json")
            config_path = os.path.join(tmp, "config.yaml")
            Path(config_path).write_text(
                "default_width: 4\nthreshold: 127\nblur_ksize: 1\nobstacle_ratio: 0.5\ngrid_size: 0.25\n",
                encoding="utf-8",
            )

            completed = subprocess.run(
                [
                    sys.executable,
                    "-m",
                    "map_pipeline.json_pipeline",
                    image_path,
                    "--config",
                    config_path,
                    "--start",
                    "0,0",
                    "--goal",
                    "3,2",
                    "--output",
                    output_path,
                ],
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )

            self.assertEqual(completed.returncode, 0, completed.stderr)
            result = read_result(output_path)
            self.assertEqual(result["schema_version"], "1.0")
            self.assertEqual(result["status"], STATUS_SUCCESS)
            self.assertEqual(result["outputs"]["map"]["start"], {"row": 0, "col": 0})
            self.assertEqual(result["outputs"]["map"]["goal"], {"row": 3, "col": 2})
            preview_path = result["outputs"]["preview"]["path"]
            self.assertTrue(os.path.exists(preview_path))
            preview = cv2.imread(preview_path)
            self.assertIsNotNone(preview)
            self.assertGreater(int(np.count_nonzero(preview)), 0)

    def test_headless_cli_creates_an_isolated_run_batch_when_output_is_omitted(self):
        with tempfile.TemporaryDirectory() as tmp:
            image_path = self._write_test_image(tmp)
            config_path = os.path.join(tmp, "config.yaml")
            run_root = os.path.join(tmp, "runs")
            Path(config_path).write_text(
                "default_width: 4\nthreshold: 127\nblur_ksize: 1\nobstacle_ratio: 0.5\ngrid_size: 0.25\n",
                encoding="utf-8",
            )

            completed = subprocess.run(
                [
                    sys.executable,
                    "-m",
                    "map_pipeline.json_pipeline",
                    image_path,
                    "--config",
                    config_path,
                    "--run-root",
                    run_root,
                    "--run-id",
                    "map-run-001",
                    "--start",
                    "0,0",
                    "--goal",
                    "3,2",
                ],
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )

            self.assertEqual(completed.returncode, 0, completed.stderr)
            result_path = os.path.join(run_root, "map-run-001", "map_pipeline", "result.json")
            manifest_path = os.path.join(run_root, "map-run-001", "manifest.json")
            result = read_result(result_path)
            manifest = json.loads(Path(manifest_path).read_text(encoding="utf-8"))
            self.assertEqual(result["run_id"], "map-run-001")
            self.assertEqual(manifest["run_id"], "map-run-001")
            self.assertEqual(manifest["status"], STATUS_SUCCESS)
            self.assertEqual(manifest["stages"]["map_pipeline"]["status"], STATUS_SUCCESS)

    def test_headless_cli_requires_both_explicit_points_when_stdout_is_not_a_tty(self):
        with tempfile.TemporaryDirectory() as tmp:
            image_path = self._write_test_image(tmp)
            completed = subprocess.run(
                [
                    sys.executable,
                    "-m",
                    "map_pipeline.json_pipeline",
                    image_path,
                ],
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )

            self.assertEqual(completed.returncode, 2)
            self.assertIn("必须同时提供", completed.stderr)

    @staticmethod
    def _config():
        return {
            "default_width": 4,
            "threshold": 127,
            "blur_ksize": 1,
            "obstacle_ratio": 0.5,
            "grid_size": 0.25,
        }

    @staticmethod
    def _write_test_image(tmp):
        image = np.full((40, 40, 3), 255, dtype=np.uint8)
        image[30:40, 30:40] = 0
        path = os.path.join(tmp, "map.png")
        cv2.imwrite(path, image)
        return path


class InteractivePickerTests(unittest.TestCase):
    @patch("map_pipeline.src.interactive_picker.cv2")
    def test_escape_cancels_selection(self, cv2_mock):
        cv2_mock.waitKey.return_value = 27
        cv2_mock.getWindowProperty.return_value = 1

        with self.assertRaises(PointSelectionCancelled):
            pick_points(np.zeros((20, 20, 3), dtype=np.uint8))

        cv2_mock.destroyWindow.assert_called_once()


if __name__ == "__main__":
    unittest.main()
