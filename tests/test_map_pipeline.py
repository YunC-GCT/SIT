import json
import os
import subprocess
import sys
import tempfile
import unittest
from typing import Callable, ClassVar
from pathlib import Path
from unittest.mock import patch

from types import SimpleNamespace

import cv2
import numpy as np

from core import STATUS_FAILED, STATUS_SUCCESS, read_result
from map_pipeline import json_pipeline
from map_pipeline import run_pipeline as legacy_map_cli
from map_pipeline.interaction import (
    _parse_grid_point,
    _parse_pixel_pair,
    _prompt_corners_terminal,
    _resolve_corners,
    _select_terminal_points,
)
from map_pipeline.cli import (
    _load_config,
    _pair,
    _positive_float,
    _positive_int,
)
from map_pipeline.map_pipeline import MapPipeline
from map_pipeline.process import _validate_pixel_point, prepare_grid_map, process
from map_pipeline.run_pipeline import _find_image, draw_grid_preview
from map_pipeline.src.coordinate_mapper import (
    map_pixel_to_grid,
    validate_grid_points,
)
from map_pipeline.src.grid_resolution_tuner import tune_resolution
from map_pipeline.src.grid_map import GridMap
from map_pipeline.src.interactive_picker import (
    PointSelectionCancelled,
    _display_geometry,
    _is_confirm_key,
    _normalize_key,
    _snap_to_grid_center,
    pick_points,
    pick_quad,
)
from map_pipeline.src.picker_common import _to_rgb, _write_temp_ppm
from map_pipeline.src.picker_tk import _pick_points_tk
from map_pipeline.src.map_loader import clean_binary, create_grid_map, load_image


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

    def test_cli_accepts_pixel_coordinates_and_rejects_bad_argument_pairs(self):
        with tempfile.TemporaryDirectory() as tmp:
            image = np.full((20, 20, 3), 255, dtype=np.uint8)
            image[12:18, 12:18] = 0
            image_path = os.path.join(tmp, "map.png")
            cv2.imwrite(image_path, image)
            config_path = os.path.join(tmp, "config.yaml")
            Path(config_path).write_text(
                "default_width: 10\n"
                "threshold: 127\n"
                "blur_ksize: 1\n"
                "obstacle_ratio: 0.5\n"
                "grid_size: 0.3\n",
                encoding="utf-8",
            )
            output_path = os.path.join(tmp, "pixel-result.json")

            exit_code = json_pipeline.main(
                [
                    image_path,
                    "--start-px",
                    "1,1",
                    "--goal-px",
                    "18,18",
                    "--config",
                    config_path,
                    "--output",
                    output_path,
                    "--run-root",
                    tmp,
                    "--run-id",
                    "pixel-001",
                ]
            )
            result = read_result(output_path)

            with self.assertRaises(SystemExit):
                json_pipeline.main(
                    [
                        image_path,
                        "--start",
                        "0,0",
                        "--run-root",
                        tmp,
                        "--run-id",
                        "bad-args",
                    ]
                )
            with self.assertRaises(SystemExit):
                json_pipeline.main(
                    [
                        image_path,
                        "--start",
                        "0,0",
                        "--goal",
                        "1,1",
                        "--start-px",
                        "0,0",
                        "--goal-px",
                        "1,1",
                        "--run-root",
                        tmp,
                        "--run-id",
                        "bad-args-2",
                    ]
                )

        self.assertEqual(exit_code, 0)
        self.assertEqual(result["status"], STATUS_SUCCESS)
        self.assertEqual(result["inputs"]["coordinate"], "pixel")

    @patch("map_pipeline.cli._select_terminal_points", return_value=((1, 1), (2, 2)))
    @patch("map_pipeline.cli.interactive_terminal", return_value=True)
    def test_cli_terminal_selection_branch(self, _interactive, _select):
        with tempfile.TemporaryDirectory() as tmp:
            image = np.full((10, 10, 3), 255, dtype=np.uint8)
            image_path = os.path.join(tmp, "map.png")
            cv2.imwrite(image_path, image)
            config_path = os.path.join(tmp, "config.yaml")
            Path(config_path).write_text(
                "default_width: 5\n"
                "threshold: 127\n"
                "blur_ksize: 1\n"
                "obstacle_ratio: 0.5\n"
                "grid_size: 0.3\n",
                encoding="utf-8",
            )
            output_path = os.path.join(tmp, "result.json")

            exit_code = json_pipeline.main(
                [
                    image_path,
                    "--select-terminal",
                    "--config",
                    config_path,
                    "--output",
                    output_path,
                    "--run-root",
                    tmp,
                    "--run-id",
                    "select-001",
                ]
            )
            result = read_result(output_path)

        self.assertEqual(exit_code, 0)
        self.assertEqual(result["status"], STATUS_SUCCESS)

    @patch(
        "map_pipeline.cli._resolve_corners",
        side_effect=ValueError("corners required"),
    )
    def test_cli_rectify_error_is_a_structured_failure(self, _resolve):
        with tempfile.TemporaryDirectory() as tmp:
            image = np.full((10, 10, 3), 255, dtype=np.uint8)
            image_path = os.path.join(tmp, "map.png")
            cv2.imwrite(image_path, image)
            output_path = os.path.join(tmp, "result.json")

            exit_code = json_pipeline.main(
                [
                    image_path,
                    "--rectify",
                    "--start",
                    "0,0",
                    "--goal",
                    "5,5",
                    "--output",
                    output_path,
                    "--run-root",
                    tmp,
                    "--run-id",
                    "rectify-failure",
                ]
            )
            result = read_result(output_path)

        self.assertEqual(exit_code, 2)
        self.assertEqual(result["status"], STATUS_FAILED)
        self.assertIn("corners required", result["errors"][0]["reason"])

    @patch(
        "map_pipeline.run_pipeline._select_terminal_points",
        return_value=((1, 1), (2, 2)),
    )
    @patch("map_pipeline.run_pipeline.interactive_terminal", return_value=True)
    def test_legacy_run_pipeline_cli_terminal_selection(self, _interactive, _select):
        with tempfile.TemporaryDirectory() as tmp:
            image = np.full((10, 10, 3), 255, dtype=np.uint8)
            image_path = os.path.join(tmp, "map.png")
            cv2.imwrite(image_path, image)
            config_path = os.path.join(tmp, "config.yaml")
            Path(config_path).write_text(
                "default_width: 5\n"
                "threshold: 127\n"
                "blur_ksize: 1\n"
                "obstacle_ratio: 0.5\n"
                "grid_size: 0.3\n",
                encoding="utf-8",
            )
            output_path = os.path.join(tmp, "legacy-result.json")

            exit_code = legacy_map_cli.main(
                [
                    "--image",
                    image_path,
                    "--select-terminal",
                    "--config",
                    config_path,
                    "--output",
                    output_path,
                    "--run-root",
                    tmp,
                    "--run-id",
                    "legacy-cli",
                ]
            )
            result = read_result(output_path)

        self.assertEqual(exit_code, 0)
        self.assertEqual(result["status"], STATUS_SUCCESS)

    def test_legacy_map_pipeline_processes_and_saves_outputs(self):
        with tempfile.TemporaryDirectory() as tmp:
            image = np.full((20, 20, 3), 255, dtype=np.uint8)
            image[12:18, 12:18] = 0
            image_path = os.path.join(tmp, "map.png")
            cv2.imwrite(image_path, image)

            pipeline = MapPipeline(
                {
                    "default_width": 10,
                    "threshold": 127,
                    "blur_ksize": 1,
                    "obstacle_ratio": 0.5,
                    "grid_size": 0.3,
                }
            )
            grid_map, start, goal = pipeline.process(
                image_path,
                (1, 1),
                (18, 18),
            )
            result_path = pipeline.save_outputs(
                grid_map,
                start,
                goal,
                tmp,
                run_id="legacy-001",
            )
            result = read_result(result_path)

            with self.assertRaises(ValueError):
                pipeline.process(image_path, (999, 1), (18, 18))

        self.assertEqual(result["status"], STATUS_SUCCESS)
        self.assertEqual(result["run_id"], "legacy-001")

    def test_grid_resolution_tuner_accepts_enter_number_and_quit(self):
        preview_calls = []

        def preview(width):
            preview_calls.append(width)
            return 10, width

        with patch("builtins.input", return_value=""):
            self.assertEqual(
                tune_resolution(20, 100, 100, preview),
                20,
            )
        with patch("builtins.input", side_effect=["40", ""]):
            self.assertEqual(
                tune_resolution(20, 100, 100, preview),
                40,
            )
        with patch("builtins.input", return_value="q"):
            with self.assertRaises(SystemExit):
                tune_resolution(20, 100, 100, preview)
        with patch("builtins.input", side_effect=["bad"] * 5):
            with self.assertRaises(SystemExit):
                tune_resolution(20, 100, 100, preview)
        self.assertTrue(preview_calls)

    def test_grid_map_free_lookup_nearest_save_and_load(self):
        with tempfile.TemporaryDirectory() as tmp:
            grid = np.array(
                [[1, 1, 1], [1, 0, 0], [1, 0, 0]],
                dtype=np.uint8,
            )
            grid_map = GridMap(grid, 2, 2, (6, 6), grid_size=0.25)

            self.assertFalse(grid_map.is_free(-1, 0))
            self.assertFalse(grid_map.is_free(0, 0))
            self.assertTrue(grid_map.is_free(1, 1))
            self.assertEqual(grid_map.find_nearest_free(0, 0), (1, 1))

            path = os.path.join(tmp, "map.npz")
            grid_map.save(path)
            loaded = GridMap.load(path)
            np.testing.assert_array_equal(loaded.grid, grid)
            self.assertEqual(loaded.grid_size, 0.25)

        with self.assertRaises(RuntimeError):
            GridMap(
                np.ones((2, 2), dtype=np.uint8),
                1,
                1,
                (2, 2),
            ).find_nearest_free(0, 0)

    def test_pixel_point_validation_and_process_failure_paths(self):
        with self.assertRaises(ValueError):
            _validate_pixel_point((1,), (10, 10, 3), "start")
        with self.assertRaises(ValueError):
            _validate_pixel_point((1.5, 2), (10, 10, 3), "start")
        with self.assertRaises(ValueError):
            _validate_pixel_point((10, 0), (10, 10, 3), "start")

        with tempfile.TemporaryDirectory() as tmp:
            image = np.full((20, 20, 3), 255, dtype=np.uint8)
            image[0:10, 0:10] = 0
            image_path = os.path.join(tmp, "obstacle.png")
            cv2.imwrite(image_path, image)

            result = process(
                image_path,
                start=(0, 0),
                goal=(1, 1),
                coordinate="grid",
                output_path=os.path.join(tmp, "result.json"),
                run_id="obstacle-001",
            )

        self.assertEqual(result["status"], STATUS_FAILED)
        self.assertEqual(result["errors"][0]["code"], "MAP_PROCESSING_ERROR")

    def test_map_cli_helpers_parse_and_normalize_values(self):
        self.assertEqual(_pair("1,2"), (1, 2))
        with self.assertRaises(Exception):
            _pair("bad")
        self.assertEqual(_positive_float("0.5"), 0.5)
        self.assertIsNone(_positive_float("bad"))
        self.assertIsNone(_positive_float(-1))
        self.assertEqual(_positive_int("3"), 3)
        self.assertIsNone(_positive_int("0"))

        with tempfile.TemporaryDirectory() as tmp:
            config_path = os.path.join(tmp, "map.yaml")
            Path(config_path).write_text(
                "map_pipeline:\n  default_width: 40\n  grid_size: 0.2\n",
                encoding="utf-8",
            )
            config = _load_config(config_path)

        self.assertEqual(config["default_width"], 40)
        self.assertEqual(config["grid_size"], 0.2)

    def test_prepare_grid_map_clamps_default_width_for_small_images(self):
        grid_map = prepare_grid_map(
            np.full((80, 80, 3), 255, dtype=np.uint8),
            {"default_width": 500, "blur_ksize": 1},
        )

        self.assertEqual(grid_map.cols, 80)

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
            assert preview is not None
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
    def test_small_map_display_is_upscaled_to_fill_the_picker(self):
        scale, display_width, display_height = _display_geometry(80, 80)

        self.assertEqual(scale, 8.75)
        self.assertEqual((display_width, display_height), (700, 700))

    @patch("map_pipeline.src.interactive_picker._pick_points_opencv")
    @patch("map_pipeline.src.interactive_picker._pick_points_tk")
    def test_default_picker_prefers_tk_backend(self, tk_picker, opencv_picker):
        tk_picker.return_value = ((1, 2), (3, 4))

        points = pick_points(np.zeros((20, 20, 3), dtype=np.uint8))

        self.assertEqual(points, ((1, 2), (3, 4)))
        tk_picker.assert_called_once()
        opencv_picker.assert_not_called()

    @patch("map_pipeline.src.interactive_picker._pick_points_opencv")
    @patch("map_pipeline.src.interactive_picker._pick_points_tk")
    def test_tk_cancellation_is_not_retried_with_opencv(
        self,
        tk_picker,
        opencv_picker,
    ):
        tk_picker.side_effect = PointSelectionCancelled("cancelled")

        with self.assertRaises(PointSelectionCancelled):
            pick_points(np.zeros((20, 20, 3), dtype=np.uint8))

        opencv_picker.assert_not_called()

    @patch("map_pipeline.src.picker_opencv.cv2")
    def test_picker_uses_explicit_resizable_window_geometry(self, cv2_mock):
        cv2_mock.waitKeyEx.return_value = 27
        cv2_mock.getWindowProperty.return_value = 1
        cv2_mock.resize.side_effect = (
            lambda image, _size, interpolation=None: image
        )

        with self.assertRaises(PointSelectionCancelled):
            pick_points(
                np.zeros((80, 80, 3), dtype=np.uint8),
                backend="opencv",
            )

        cv2_mock.namedWindow.assert_called_once_with(
            "Select Start & Goal",
            cv2_mock.WINDOW_NORMAL,
        )
        cv2_mock.resizeWindow.assert_called_once_with(
            "Select Start & Goal",
            700,
            700,
        )

    def test_extended_enter_key_is_normalized_to_confirmation(self):
        self.assertEqual(_normalize_key(0x1000000D), 13)
        self.assertEqual(_normalize_key(0x1000000A), 10)
        self.assertTrue(_is_confirm_key(0x01000004))
        self.assertTrue(_is_confirm_key(0x01000005))
        self.assertFalse(_is_confirm_key(ord("d")))

    @patch("map_pipeline.src.picker_opencv.cv2")
    def test_extended_enter_confirms_two_selected_points(self, cv2_mock):
        cv2_mock.EVENT_LBUTTONDOWN = 1
        cv2_mock.waitKeyEx.return_value = 0x1000000D
        cv2_mock.getWindowProperty.return_value = 1
        cv2_mock.resize.side_effect = (
            lambda image, _size, interpolation=None: image
        )

        def select_two_points(_window, callback):
            callback(1, 100, 100, 0, None)
            callback(1, 500, 500, 0, None)

        cv2_mock.setMouseCallback.side_effect = select_two_points

        points = pick_points(
            np.zeros((20, 20, 3), dtype=np.uint8),
            backend="opencv",
        )

        self.assertEqual(points, ((2, 2), (14, 14)))
        cv2_mock.destroyWindow.assert_called_once()

    @patch("map_pipeline.src.picker_opencv.cv2")
    def test_picker_snaps_selected_points_to_grid_centers(self, cv2_mock):
        cv2_mock.EVENT_LBUTTONDOWN = 1
        cv2_mock.waitKeyEx.return_value = 0x1000000D
        cv2_mock.getWindowProperty.return_value = 1
        cv2_mock.resize.side_effect = (
            lambda image, _size, interpolation=None: image
        )

        grid_map = GridMap(
            np.zeros((2, 2), dtype=np.uint8),
            cell_width=10,
            cell_height=10,
            img_shape=(20, 20),
        )

        def select_two_points(_window, callback):
            callback(1, 100, 100, 0, None)
            callback(1, 500, 500, 0, None)

        cv2_mock.setMouseCallback.side_effect = select_two_points

        points = pick_points(
            np.zeros((20, 20, 3), dtype=np.uint8),
            backend="opencv",
            grid_map=grid_map,
        )

        self.assertEqual(points, ((5, 5), (15, 15)))
        cv2_mock.destroyWindow.assert_called_once()

    @patch("map_pipeline.src.picker_quad.cv2")
    def test_quad_picker_confirms_four_clicks(self, cv2_mock):
        cv2_mock.EVENT_LBUTTONDOWN = 1
        cv2_mock.waitKeyEx.return_value = 0x1000000D
        cv2_mock.getWindowProperty.return_value = 1

        def select_four(_window, callback):
            for x, y in ((100, 100), (500, 100), (500, 500), (100, 500)):
                callback(1, x, y, 0, None)

        cv2_mock.setMouseCallback.side_effect = select_four

        points = pick_quad(np.zeros((20, 20, 3), dtype=np.uint8))

        self.assertEqual(points, [(2, 2), (14, 2), (14, 14), (2, 14)])
        cv2_mock.destroyWindow.assert_called_once()

    @patch("map_pipeline.src.picker_quad.cv2")
    def test_quad_picker_escape_cancels(self, cv2_mock):
        cv2_mock.waitKeyEx.return_value = 27
        cv2_mock.getWindowProperty.return_value = 1

        with self.assertRaises(PointSelectionCancelled):
            pick_quad(np.zeros((20, 20, 3), dtype=np.uint8))

        cv2_mock.destroyWindow.assert_called_once()

    def test_rgb_conversion_and_ppm_writer(self):
        self.assertEqual(
            _to_rgb(np.zeros((2, 2), dtype=np.uint8)).shape,
            (2, 2, 3),
        )
        self.assertEqual(
            _to_rgb(np.zeros((2, 2, 4), dtype=np.uint8)).shape,
            (2, 2, 3),
        )
        path = _write_temp_ppm(np.zeros((2, 3, 3), dtype=np.uint8))
        try:
            data = Path(path).read_bytes()
            self.assertTrue(data.startswith(b"P6\n3 2\n255\n"))
        finally:
            os.unlink(path)

    def test_grid_center_snap_stays_in_one_pixel_cells(self):
        grid_map = GridMap(
            np.zeros((2, 2), dtype=np.uint8),
            cell_width=1,
            cell_height=1,
            img_shape=(2, 2),
        )

        point = _snap_to_grid_center((1, 1), grid_map)

        self.assertEqual(point, (1, 1))
        self.assertEqual(map_pixel_to_grid(*point, grid_map), (1, 1))

    @patch("map_pipeline.src.picker_opencv.cv2")
    def test_escape_cancels_selection(self, cv2_mock):
        cv2_mock.waitKeyEx.return_value = 27
        cv2_mock.getWindowProperty.return_value = 1

        with self.assertRaises(PointSelectionCancelled):
            pick_points(
                np.zeros((20, 20, 3), dtype=np.uint8),
                backend="opencv",
            )

        cv2_mock.destroyWindow.assert_called_once()


    def test_rectified_cli_crops_and_calibrates_grid_size(self):
        with tempfile.TemporaryDirectory() as tmp:
            image = np.full((60, 60, 3), 255, dtype=np.uint8)
            image[20:40, 20:40] = 0
            image_path = os.path.join(tmp, "map.png")
            cv2.imwrite(image_path, image)
            config_path = os.path.join(tmp, "config.yaml")
            Path(config_path).write_text(
                "default_width: 30\n"
                "threshold: 127\n"
                "blur_ksize: 1\n"
                "obstacle_ratio: 0.5\n"
                "grid_size: 0.1\n",
                encoding="utf-8",
            )

            exit_code = json_pipeline.main(
                [
                    image_path,
                    "--rectify",
                    "--corners",
                    "0,0;59,0;59,59;0,59",
                    "--arena-width-m",
                    "3.0",
                    "--rectify-output-width",
                    "120",
                    "--start",
                    "1,1",
                    "--goal",
                    "3,3",
                    "--config",
                    config_path,
                    "--run-root",
                    tmp,
                    "--run-id",
                    "rect-001",
                    "--headless",
                ]
            )

            self.assertEqual(exit_code, 0)
            result = read_result(
                os.path.join(tmp, "rect-001", "map_pipeline", "result.json")
            )
            self.assertEqual(result["status"], STATUS_SUCCESS)
            self.assertEqual(
                result["outputs"]["rectification"]["output_size"]["width"],
                120,
            )
            self.assertTrue(
                result["outputs"]["rectified_image"]["path"].endswith(".png")
            )
            self.assertAlmostEqual(
                result["outputs"]["map"]["grid_size_m"],
                3.0 / 30,
            )


    def test_terminal_corner_input_validates_format_and_bounds(self):
        answers = iter(["bad", "999,999", "10,10", "20,10", "20,20", "10,20"])
        messages = []

        corners = _prompt_corners_terminal(
            (100, 100),
            input_fn=lambda _prompt: next(answers),
            output_fn=messages.append,
        )

        self.assertEqual(
            corners,
            [(10, 10), (20, 10), (20, 20), (10, 20)],
        )
        text = "\n".join(messages)
        self.assertIn("格式应为", text)
        self.assertIn("超出范围", text)

    def test_explicit_corners_are_resolved_without_gui(self):
        image = np.zeros((100, 100, 3), dtype=np.uint8)
        args = SimpleNamespace(
            corners="10,10;20,10;20,20;10,20",
        )

        corners = _resolve_corners(args, image)

        self.assertEqual(len(corners), 4)
        self.assertEqual(corners[0], (10.0, 10.0))
        self.assertEqual(corners[2], (20.0, 20.0))

    def test_grid_and_pixel_pair_parsers(self):
        self.assertEqual(_parse_grid_point("1, 2"), (1, 2))
        self.assertEqual(_parse_grid_point("1，2"), (1, 2))
        self.assertEqual(_parse_pixel_pair("3,4"), (3, 4))
        with self.assertRaises(ValueError):
            _parse_grid_point("1")
        with self.assertRaises(ValueError):
            _parse_pixel_pair("x,4")

    def test_morphology_cleanup_and_image_loader_errors(self):
        binary = np.full((20, 20), 255, dtype=np.uint8)
        binary[5:15, 5:15] = 0
        binary[0, 0] = 0

        cleaned = clean_binary(binary, morphology_kernel=3)

        self.assertEqual(cleaned[0, 0], 255)
        self.assertTrue(np.all(cleaned[5:15, 5:15] == 0))
        with self.assertRaises(FileNotFoundError):
            load_image(os.path.join(tempfile.gettempdir(), "missing-map.png"))
        with self.assertRaises(ValueError):
            create_grid_map(np.zeros((4, 4), dtype=np.uint8), 0)
        with self.assertRaises(ValueError):
            create_grid_map(np.zeros((4, 4), dtype=np.uint8), 4, obstacle_ratio=2.0)


class TkPickerTests(unittest.TestCase):
    class FakeRoot:
        instances = []
        scenario: ClassVar[Callable[["TkPickerTests.FakeRoot"], None] | None] = None

        def __init__(self):
            self.bindings = {}
            self.protocols = {}
            self.destroyed = False
            type(self).instances.append(self)

        def title(self, *_args, **_kwargs):
            pass

        def resizable(self, *_args, **_kwargs):
            pass

        def bind_all(self, key, callback):
            self.bindings[key] = callback

        def protocol(self, name, callback):
            self.protocols[name] = callback

        def quit(self):
            pass

        def update_idletasks(self):
            pass

        def focus_force(self):
            pass

        def mainloop(self):
            scenario = type(self).scenario
            if scenario is not None:
                scenario(self)

        def destroy(self):
            self.destroyed = True

    class FakeCanvas:
        instances = []

        def __init__(self, *_args, **_kwargs):
            self.bindings = {}
            self.items = []
            type(self).instances.append(self)

        def pack(self):
            pass

        def create_image(self, *args, **kwargs):
            self.items.append(("image", args, kwargs))

        def create_line(self, *args, **kwargs):
            self.items.append(("line", args, kwargs))

        def create_oval(self, *args, **kwargs):
            self.items.append(("oval", args, kwargs))

        def create_text(self, *args, **kwargs):
            self.items.append(("text", args, kwargs))

        def create_rectangle(self, *args, **kwargs):
            self.items.append(("rectangle", args, kwargs))

        def delete(self, *_args, **_kwargs):
            pass

        def bind(self, key, callback):
            self.bindings[key] = callback

        def focus_set(self):
            pass

    class FakePhotoImage:
        def __init__(self, file=None):
            self.file = file

    def setUp(self):
        self.FakeRoot.instances.clear()
        self.FakeCanvas.instances.clear()
        self.FakeRoot.scenario = None

    def fake_tkinter(self):
        return SimpleNamespace(
            Tk=self.FakeRoot,
            Canvas=self.FakeCanvas,
            PhotoImage=self.FakePhotoImage,
            NW="nw",
            W="w",
        )

    def test_tk_picker_confirms_two_clicks(self):
        def scenario(root):
            canvas = self.FakeCanvas.instances[-1]
            left = canvas.bindings["<Button-1>"]
            left(SimpleNamespace(x=100, y=100))
            left(SimpleNamespace(x=500, y=500))
            root.bindings["<Return>"](None)

        self.FakeRoot.scenario = scenario
        with patch.dict(sys.modules, {"tkinter": self.fake_tkinter()}):
            points = _pick_points_tk(
                np.zeros((20, 20, 3), dtype=np.uint8),
                "Select",
                None,
            )

        self.assertEqual(points, ((2, 2), (14, 14)))
        self.assertTrue(self.FakeRoot.instances[-1].destroyed)

    def test_tk_picker_escape_and_window_close_cancel(self):
        self.FakeRoot.scenario = lambda root: root.bindings["<Escape>"](None)
        with patch.dict(sys.modules, {"tkinter": self.fake_tkinter()}):
            with self.assertRaises(PointSelectionCancelled):
                _pick_points_tk(
                    np.zeros((20, 20, 3), dtype=np.uint8),
                    "Select",
                    None,
                )

        self.FakeRoot.instances.clear()
        self.FakeCanvas.instances.clear()
        self.FakeRoot.scenario = lambda root: root.protocols[
            "WM_DELETE_WINDOW"
        ]()
        with patch.dict(sys.modules, {"tkinter": self.fake_tkinter()}):
            with self.assertRaises(PointSelectionCancelled):
                _pick_points_tk(
                    np.zeros((20, 20, 3), dtype=np.uint8),
                    "Select",
                    None,
                )

    def test_tk_picker_rejects_obstacle_and_supports_undo(self):
        grid_map = GridMap(
            np.array([[0, 1], [0, 0]], dtype=np.uint8),
            cell_width=10,
            cell_height=10,
            img_shape=(20, 20),
        )

        def scenario(root):
            canvas = self.FakeCanvas.instances[-1]
            left = canvas.bindings["<Button-1>"]
            right = canvas.bindings["<Button-3>"]
            left(SimpleNamespace(x=525, y=175))
            left(SimpleNamespace(x=175, y=175))
            root.bindings["<Return>"](None)
            right(None)
            right(None)
            left(SimpleNamespace(x=175, y=175))
            left(SimpleNamespace(x=175, y=525))
            root.bindings["<Return>"](None)

        self.FakeRoot.scenario = scenario
        with patch.dict(sys.modules, {"tkinter": self.fake_tkinter()}):
            points = _pick_points_tk(
                np.zeros((20, 20, 3), dtype=np.uint8),
                "Select",
                grid_map,
            )

        self.assertEqual(points, ((5, 5), (5, 15)))


class LegacyRunPipelineTests(unittest.TestCase):
    def _fixture(self, tmp):
        image = np.full((10, 10, 3), 255, dtype=np.uint8)
        image_path = os.path.join(tmp, "map.png")
        cv2.imwrite(image_path, image)
        config_path = os.path.join(tmp, "config.yaml")
        Path(config_path).write_text(
            "default_width: 5\n"
            "threshold: 127\n"
            "blur_ksize: 1\n"
            "obstacle_ratio: 0.5\n"
            "grid_size: 0.3\n",
            encoding="utf-8",
        )
        return image_path, config_path

    def test_find_image_prompts_for_single_file_in_tty(self):
        with tempfile.TemporaryDirectory() as tmp:
            Path(tmp, "only.png").write_bytes(b"image")

            with patch(
                "map_pipeline.run_pipeline.sys.stdin.isatty",
                return_value=True,
            ), patch("builtins.input", return_value="1") as prompt:
                path = _find_image(tmp, None)
            self.assertTrue(path.endswith("only.png"))
            prompt.assert_called_once()

            with patch(
                "map_pipeline.run_pipeline.sys.stdin.isatty",
                return_value=True,
            ), patch("builtins.input", return_value=""):
                path = _find_image(tmp, None)
            self.assertTrue(path.endswith("only.png"))

            with patch(
                "map_pipeline.run_pipeline.sys.stdin.isatty",
                return_value=False,
            ), patch(
                "builtins.input",
                side_effect=AssertionError("should not prompt"),
            ):
                path = _find_image(tmp, None)
            self.assertTrue(path.endswith("only.png"))

    def test_find_image_prompts_and_validates_choice(self):
        temporary_maps = tempfile.TemporaryDirectory()
        self.addCleanup(temporary_maps.cleanup)
        input_dir = temporary_maps.name
        for name in ("first.png", "second.png"):
            Path(input_dir, name).touch()
        with patch(
            "map_pipeline.run_pipeline.sys.stdin.isatty",
            return_value=True,
        ), patch("builtins.input", return_value="2"):
            path = _find_image(input_dir, None)
        names = sorted(
            name for name in os.listdir(input_dir)
            if name.lower().endswith((".png", ".jpg", ".jpeg", ".bmp"))
        )
        self.assertTrue(path.endswith(names[1]))

        with patch(
            "map_pipeline.run_pipeline.sys.stdin.isatty",
            return_value=True,
        ), patch("builtins.input", return_value=""):
            path = _find_image(input_dir, None)
        self.assertTrue(path.endswith(names[0]))

        with patch(
            "map_pipeline.run_pipeline.sys.stdin.isatty",
            return_value=True,
        ), patch("builtins.input", return_value="999"):
            with self.assertRaises(ValueError):
                _find_image(input_dir, None)

    def test_draw_grid_preview_with_and_without_window(self):
        grid = np.zeros((4, 4), dtype=np.uint8)
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "grid.png")
            with patch("map_pipeline.run_pipeline.cv2") as cv2_mock:
                cv2_mock.resize.return_value = np.zeros(
                    (8, 8, 3),
                    dtype=np.uint8,
                )
                draw_grid_preview(
                    grid,
                    (0, 0),
                    (3, 3),
                    path,
                    show_window=True,
                )
            cv2_mock.namedWindow.assert_called_once()
            cv2_mock.imshow.assert_called_once()
            self.assertTrue(os.path.exists(path))

            hidden = os.path.join(tmp, "grid-hidden.png")
            draw_grid_preview(
                grid,
                (0, 0),
                (3, 3),
                hidden,
                show_window=False,
            )
            self.assertTrue(os.path.exists(hidden))

    @patch(
        "map_pipeline.run_pipeline._select_terminal_points",
        side_effect=PointSelectionCancelled("cancel"),
    )
    @patch("map_pipeline.run_pipeline.interactive_terminal", return_value=True)
    def test_run_pipeline_cancel_is_structured(self, _interactive, _select):
        with tempfile.TemporaryDirectory() as tmp:
            image_path, config_path = self._fixture(tmp)
            output_path = os.path.join(tmp, "result.json")
            exit_code = legacy_map_cli.main(
                [
                    "--image",
                    image_path,
                    "--select-terminal",
                    "--config",
                    config_path,
                    "--output",
                    output_path,
                    "--run-root",
                    tmp,
                    "--run-id",
                    "legacy-cancel",
                ]
            )
            result = read_result(output_path)

        self.assertEqual(exit_code, 2)
        self.assertEqual(result["errors"][0]["code"], "POINT_SELECTION_CANCELLED")

    @patch(
        "map_pipeline.run_pipeline._select_terminal_points",
        side_effect=ValueError("boom"),
    )
    @patch("map_pipeline.run_pipeline.interactive_terminal", return_value=True)
    def test_run_pipeline_generic_failure_is_structured(self, _interactive, _select):
        with tempfile.TemporaryDirectory() as tmp:
            image_path, config_path = self._fixture(tmp)
            output_path = os.path.join(tmp, "result.json")
            exit_code = legacy_map_cli.main(
                [
                    "--image",
                    image_path,
                    "--select-terminal",
                    "--config",
                    config_path,
                    "--output",
                    output_path,
                    "--run-root",
                    tmp,
                    "--run-id",
                    "legacy-failure",
                ]
            )
            result = read_result(output_path)

        self.assertEqual(exit_code, 2)
        self.assertEqual(result["status"], STATUS_FAILED)
        self.assertIn("boom", result["errors"][0]["reason"])

    @patch("map_pipeline.run_pipeline.interactive_terminal", return_value=False)
    def test_run_pipeline_headless_without_points_fails(self, _interactive):
        with tempfile.TemporaryDirectory() as tmp:
            image_path, config_path = self._fixture(tmp)
            output_path = os.path.join(tmp, "result.json")
            exit_code = legacy_map_cli.main(
                [
                    "--image",
                    image_path,
                    "--headless",
                    "--config",
                    config_path,
                    "--output",
                    output_path,
                    "--run-root",
                    tmp,
                    "--run-id",
                    "legacy-headless",
                ]
            )
            result = read_result(output_path)

        self.assertEqual(exit_code, 2)
        self.assertEqual(result["status"], STATUS_FAILED)

    @patch(
        "map_pipeline.run_pipeline._select_terminal_points",
        return_value=((1, 1), (2, 2)),
    )
    @patch("map_pipeline.run_pipeline.interactive_terminal", return_value=True)
    @patch("map_pipeline.run_pipeline.tune_resolution", return_value=6)
    def test_run_pipeline_adjust_grid_branch(self, _tune, _interactive, _select):
        with tempfile.TemporaryDirectory() as tmp:
            image_path, config_path = self._fixture(tmp)
            output_path = os.path.join(tmp, "result.json")
            exit_code = legacy_map_cli.main(
                [
                    "--image",
                    image_path,
                    "--select-terminal",
                    "--adjust-grid",
                    "--config",
                    config_path,
                    "--output",
                    output_path,
                    "--run-root",
                    tmp,
                    "--run-id",
                    "legacy-adjust",
                ]
            )
            result = read_result(output_path)

        self.assertEqual(exit_code, 0)
        self.assertEqual(result["status"], STATUS_SUCCESS)


class TerminalSelectionTests(unittest.TestCase):
    def test_terminal_selection_validates_input_and_avoids_obstacles(self):
        grid = np.zeros((4, 4), dtype=np.uint8)
        grid[1, 1] = 1
        grid_map = GridMap(grid, 1, 1, (4, 4), grid_size=0.3)
        answers = iter(["9,9", "abc", "1,1", "0,0", "0,0", "0,3"])
        messages = []

        with tempfile.TemporaryDirectory() as tmp:
            preview_path = os.path.join(tmp, "preview.png")
            start, goal = _select_terminal_points(
                grid_map,
                preview_path,
                input_fn=lambda _prompt: next(answers),
                output_fn=messages.append,
            )
            self.assertTrue(os.path.exists(preview_path))

        self.assertEqual((start, goal), ((0, 0), (0, 3)))
        text = "\n".join(messages)
        self.assertIn("超出范围", text)
        self.assertIn("格式应为", text)
        self.assertIn("障碍", text)
        self.assertIn("终点不能与起点相同", text)

    def test_terminal_selection_cancel_raises(self):
        grid_map = GridMap(np.zeros((3, 3), dtype=np.uint8), 1, 1, (3, 3))
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(PointSelectionCancelled):
                _select_terminal_points(
                    grid_map,
                    os.path.join(tmp, "preview.png"),
                    input_fn=lambda _prompt: "",
                    output_fn=lambda _text: None,
                )


if __name__ == "__main__":
    unittest.main()
