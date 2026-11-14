import unittest

import cv2
import numpy as np

from map_pipeline.src.map_loader import clean_binary
from map_pipeline.src.rectify import (
    MAX_RECTIFIED_DIMENSION,
    RectificationError,
    parse_corners,
    rectify_image,
    transform_points,
    validate_corners,
)


class RectifyTests(unittest.TestCase):
    def test_parse_corners_requires_four_pairs(self):
        self.assertEqual(
            parse_corners("0,0;10,0;10,10;0,10"),
            [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)],
        )
        self.assertEqual(
            parse_corners("0,0|10,0|10,10|0,10"),
            [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)],
        )
        with self.assertRaises(RectificationError):
            parse_corners("0,0;10,0;10,10")
        with self.assertRaises(RectificationError):
            parse_corners("0,0;10,0;10,10;bad")

    def test_validate_corners_orders_and_rejects_out_of_bounds(self):
        ordered = validate_corners(
            [(10, 10), (0, 10), (10, 0), (0, 0)],
            (20, 20),
        )
        self.assertEqual(ordered[0], (0.0, 0.0))
        self.assertEqual(ordered[1], (10.0, 0.0))
        self.assertEqual(ordered[2], (10.0, 10.0))
        self.assertEqual(ordered[3], (0.0, 10.0))
        with self.assertRaises(RectificationError):
            validate_corners([(0, 0), (30, 0), (10, 10), (0, 10)], (20, 20))

    def test_rotated_square_corners_are_ordered_without_duplicates(self):
        ordered = validate_corners(
            [(50, 0), (100, 50), (50, 100), (0, 50)],
            (120, 120),
        )

        self.assertEqual(len(set(ordered)), 4)
        self.assertEqual(ordered[0], (50.0, 0.0))
        self.assertEqual(ordered[1], (100.0, 50.0))
        self.assertEqual(ordered[2], (50.0, 100.0))
        self.assertEqual(ordered[3], (0.0, 50.0))

    def test_arena_height_requires_arena_width(self):
        image = np.full((100, 100, 3), 255, dtype=np.uint8)
        with self.assertRaises(RectificationError):
            rectify_image(
                image,
                [(0, 0), (99, 0), (99, 99), (0, 99)],
                arena_height_m=1.0,
            )

    def test_rectified_output_dimension_is_bounded(self):
        image = np.full((100, 100, 3), 255, dtype=np.uint8)
        with self.assertRaises(RectificationError):
            rectify_image(
                image,
                [(0, 0), (99, 0), (99, 99), (0, 99)],
                output_width=MAX_RECTIFIED_DIMENSION + 1,
            )

    def test_rectify_image_warps_a_trapezoid_back_to_a_rectangle(self):
        source = np.full((100, 100, 3), 255, dtype=np.uint8)
        source[25:75, 25:75] = 0
        source_quad = np.float32([[0, 0], [99, 0], [99, 99], [0, 99]])
        photo_quad = np.float32([[20, 10], [90, 0], [99, 95], [5, 80]])
        homography = cv2.getPerspectiveTransform(source_quad, photo_quad)
        photo = cv2.warpPerspective(
            source,
            homography,
            (120, 110),
            borderValue=(255, 255, 255),
        )

        rectified, metadata = rectify_image(
            photo,
            photo_quad,
            output_width=100,
        )

        self.assertEqual(rectified.shape[1], 100)
        self.assertGreaterEqual(rectified.shape[0], 90)
        self.assertLessEqual(rectified.shape[0], 110)
        black_ratio = float(np.mean(rectified < 128))
        self.assertGreater(black_ratio, 0.15)
        self.assertLess(black_ratio, 0.35)
        self.assertEqual(metadata["output_size"]["width"], 100)
        self.assertEqual(len(metadata["source_corners"]), 4)
        self.assertEqual(len(metadata["homography"]), 3)

    def test_aspect_ratio_uses_arena_dimensions_when_available(self):
        image = np.full((100, 100, 3), 255, dtype=np.uint8)
        rectified, metadata = rectify_image(
            image,
            [(0, 0), (99, 0), (99, 99), (0, 99)],
            output_width=100,
            arena_width_m=2.0,
            arena_height_m=1.0,
        )

        self.assertEqual(rectified.shape, (50, 100, 3))
        self.assertEqual(metadata["arena_size_m"], {"width": 2.0, "height": 1.0})
        self.assertAlmostEqual(metadata["meters_per_pixel"], 0.02)

    def test_transform_points_maps_through_homography(self):
        homography = np.eye(3).tolist()
        self.assertEqual(
            transform_points([(3, 4)], homography),
            [(3.0, 4.0)],
        )

    def test_clean_binary_removes_small_specks(self):
        binary = np.full((20, 20), 255, dtype=np.uint8)
        binary[2, 2] = 0
        binary[10:15, 10:15] = 0

        cleaned = clean_binary(binary, min_area_px=5)

        self.assertEqual(cleaned[2, 2], 255)
        self.assertTrue(np.all(cleaned[10:15, 10:15] == 0))


if __name__ == "__main__":
    unittest.main()
