"""Perspective rectification and cropping for map images."""

from __future__ import annotations

import math
import re
from typing import Sequence

import cv2
import numpy as np


class RectificationError(ValueError):
    """Raised when four-corner rectification input is invalid."""


MAX_RECTIFIED_DIMENSION = 8000


def parse_corners(text: str) -> list[tuple[float, float]]:
    """Parse ``x,y;x,y;x,y;x,y`` into four pixel coordinates."""
    if not isinstance(text, str) or not text.strip():
        raise RectificationError("corners must contain four x,y pairs")
    parts = [part for part in re.split(r"[;|]", text) if part.strip()]
    if len(parts) != 4:
        raise RectificationError(
            "corners must contain four x,y pairs separated by ';'"
        )
    corners = []
    for part in parts:
        values = part.replace("，", ",").split(",")
        if len(values) != 2:
            raise RectificationError(f"invalid corner: {part!r}")
        try:
            x, y = float(values[0]), float(values[1])
        except ValueError as exc:
            raise RectificationError(f"invalid corner: {part!r}") from exc
        if not (math.isfinite(x) and math.isfinite(y)):
            raise RectificationError(f"corner is not finite: {part!r}")
        corners.append((x, y))
    return corners


def _order_quad(points: Sequence[Sequence[float]]) -> np.ndarray:
    """Order four points as top-left, top-right, bottom-right, bottom-left."""
    array = np.asarray(points, dtype=np.float32)
    if array.shape != (4, 2):
        raise RectificationError("corners must contain four x,y points")
    if len({(float(x), float(y)) for x, y in array}) != 4:
        raise RectificationError("corners must be four distinct points")
    center = array.mean(axis=0)
    angles = np.arctan2(array[:, 1] - center[1], array[:, 0] - center[0])
    ordered = array[np.argsort(angles)]
    start = int(np.argmin(ordered.sum(axis=1)))
    ordered = np.roll(ordered, -start, axis=0)
    # Keep a clockwise sequence with the top-left-most point first.  The
    # sum/difference shortcut mis-orders rotated squares (ties in both sums and
    # differences), so sort around the centroid instead.
    if ordered[1][0] < ordered[0][0]:
        ordered = np.array(
            [ordered[0], ordered[3], ordered[2], ordered[1]],
            dtype=np.float32,
        )
    if not _is_convex(ordered):
        raise RectificationError("corners must form a convex quadrilateral")
    return ordered


def _is_convex(points: np.ndarray) -> bool:
    signs = []
    for index in range(4):
        first = points[index]
        second = points[(index + 1) % 4]
        third = points[(index + 2) % 4]
        cross = (
            (second[0] - first[0]) * (third[1] - second[1])
            - (second[1] - first[1]) * (third[0] - second[0])
        )
        signs.append(cross)
    return all(value > 0 for value in signs) or all(value < 0 for value in signs)


def validate_corners(
    corners: Sequence[Sequence[float]],
    image_shape: Sequence[int],
) -> list[tuple[float, float]]:
    height, width = int(image_shape[0]), int(image_shape[1])
    ordered = _order_quad(corners)
    result = []
    for x, y in ordered:
        if not (0 <= x < width and 0 <= y < height):
            raise RectificationError(
                f"corner ({x:.1f}, {y:.1f}) is outside the image "
                f"{width}x{height}"
            )
        result.append((float(x), float(y)))
    return result


def _side_length(first: np.ndarray, second: np.ndarray) -> float:
    return float(math.hypot(second[0] - first[0], second[1] - first[1]))


def rectify_image(
    image: np.ndarray,
    corners: Sequence[Sequence[float]],
    *,
    output_width: int | None = None,
    arena_width_m: float | None = None,
    arena_height_m: float | None = None,
) -> tuple[np.ndarray, dict]:
    """Warp a four-corner quadrilateral to a top-down rectangle."""
    ordered = validate_corners(corners, image.shape)
    points = np.asarray(ordered, dtype=np.float32)
    source_width = max(
        _side_length(points[0], points[1]),
        _side_length(points[3], points[2]),
    )
    source_height = max(
        _side_length(points[0], points[3]),
        _side_length(points[1], points[2]),
    )
    if source_width <= 0 or source_height <= 0:
        raise RectificationError("corners form a zero-area quadrilateral")

    if output_width is None:
        width = int(round(source_width))
    else:
        if not isinstance(output_width, int) or output_width < 1:
            raise RectificationError("output_width must be a positive integer")
        width = output_width
    width = max(1, width)

    if arena_width_m is not None and float(arena_width_m) <= 0:
        raise RectificationError("arena_width_m must be positive")
    if arena_height_m is not None and float(arena_height_m) <= 0:
        raise RectificationError("arena_height_m must be positive")
    if arena_height_m is not None and arena_width_m is None:
        raise RectificationError("arena_height_m requires arena_width_m")
    if width > MAX_RECTIFIED_DIMENSION:
        raise RectificationError(
            f"rectified width {width} exceeds {MAX_RECTIFIED_DIMENSION}"
        )
    if arena_width_m is not None and arena_height_m is not None:
        height = int(round(width * float(arena_height_m) / float(arena_width_m)))
    else:
        height = int(round(width * source_height / source_width))
    height = max(1, height)
    if height > MAX_RECTIFIED_DIMENSION:
        raise RectificationError(
            f"rectified height {height} exceeds {MAX_RECTIFIED_DIMENSION}"
        )

    destination = np.array(
        [
            [0.0, 0.0],
            [width - 1.0, 0.0],
            [width - 1.0, height - 1.0],
            [0.0, height - 1.0],
        ],
        dtype=np.float32,
    )
    homography = cv2.getPerspectiveTransform(points, destination)
    rectified = cv2.warpPerspective(image, homography, (width, height))

    metadata = {
        "enabled": True,
        "source_corners": [[float(x), float(y)] for x, y in ordered],
        "output_size": {"width": width, "height": height},
        "homography": homography.tolist(),
        "source_size": {
            "width": int(image.shape[1]),
            "height": int(image.shape[0]),
        },
        "arena_size_m": (
            {
                "width": float(arena_width_m) if arena_width_m is not None else None,
                "height": (
                    float(arena_height_m)
                    if arena_height_m is not None
                    else None
                ),
            }
            if arena_width_m is not None or arena_height_m is not None
            else None
        ),
        "meters_per_pixel": (
            float(arena_width_m) / width if arena_width_m is not None else None
        ),
    }
    return rectified, metadata


def transform_points(
    points: Sequence[Sequence[float]],
    homography: Sequence[Sequence[float]],
) -> list[tuple[float, float]]:
    """Map points through a homography matrix."""
    array = np.asarray([points], dtype=np.float32)
    matrix = np.asarray(homography, dtype=np.float64)
    transformed = cv2.perspectiveTransform(array, matrix)[0]
    return [(float(x), float(y)) for x, y in transformed]


__all__ = [
    "RectificationError",
    "parse_corners",
    "rectify_image",
    "transform_points",
    "validate_corners",
]
