"""Replaceable path smoothing implementations."""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from numbers import Real
from typing import Sequence

Point = tuple[float, float]


class SmoothingError(ValueError):
    """Raised when a smoother cannot produce a finite curve."""


class PathSmoother(ABC):
    """Public adapter boundary for path smoothing implementations."""

    @abstractmethod
    def smooth(
        self,
        path: Sequence[Point],
        sample_spacing_grid: float,
    ) -> list[Point]:
        """Return sampled curve points with the requested maximum spacing."""


class NaturalCubicSplineSmoother(PathSmoother):
    """Natural-boundary cubic splines parameterized by cumulative arc length."""

    def smooth(
        self,
        path: Sequence[Point],
        sample_spacing_grid: float,
    ) -> list[Point]:
        if not isinstance(sample_spacing_grid, Real) or (
            isinstance(sample_spacing_grid, bool) or sample_spacing_grid <= 0
        ):
            raise SmoothingError("sample_spacing_grid must be positive")

        points = [(float(row), float(col)) for row, col in path]
        if len(points) < 3:
            raise SmoothingError("at least three points are required")
        if any(
            not math.isfinite(value)
            for point in points
            for value in point
        ):
            raise SmoothingError("path contains a non-finite coordinate")

        distances = [0.0]
        for first, second in zip(points, points[1:]):
            distance = math.hypot(second[0] - first[0], second[1] - first[1])
            if distance <= 0:
                raise SmoothingError("path contains duplicate points")
            distances.append(distances[-1] + distance)
        total_length = distances[-1]
        if total_length <= 0 or not math.isfinite(total_length):
            raise SmoothingError("path length is not positive and finite")

        row_coefficients = self._coefficients(distances, [point[0] for point in points])
        col_coefficients = self._coefficients(distances, [point[1] for point in points])
        sample_count = max(1, math.ceil(total_length / float(sample_spacing_grid)))
        for _ in range(20):
            samples = [
                self._sample(
                    distances,
                    row_coefficients,
                    col_coefficients,
                    index / sample_count,
                    total_length,
                )
                for index in range(sample_count + 1)
            ]
            if all(
                math.hypot(
                    second[0] - first[0],
                    second[1] - first[1],
                ) <= float(sample_spacing_grid) + 1e-9
                for first, second in zip(samples, samples[1:])
            ):
                break
            sample_count *= 2
        else:
            raise SmoothingError("failed to satisfy spline sample spacing")

        samples[0] = points[0]
        samples[-1] = points[-1]
        if any(
            not math.isfinite(value)
            for point in samples
            for value in point
        ):
            raise SmoothingError("spline evaluation produced a non-finite value")
        return samples

    @classmethod
    def _sample(
        cls,
        knots: Sequence[float],
        row_coefficients: tuple[list[float], list[float], list[float], list[float]],
        col_coefficients: tuple[list[float], list[float], list[float], list[float]],
        fraction: float,
        total_length: float,
    ) -> Point:
        arc_length = total_length * fraction
        return (
            cls._evaluate(knots, row_coefficients, arc_length),
            cls._evaluate(knots, col_coefficients, arc_length),
        )

    @staticmethod
    def _coefficients(
        knots: Sequence[float],
        values: Sequence[float],
    ) -> tuple[list[float], list[float], list[float], list[float]]:
        count = len(values)
        if count < 3 or len(knots) != count:
            raise SmoothingError("natural cubic spline needs at least three knots")

        intervals = [
            second - first for first, second in zip(knots, knots[1:])
        ]
        if any(interval <= 0 for interval in intervals):
            raise SmoothingError("spline knots must be strictly increasing")

        alpha = [0.0] * count
        for index in range(1, count - 1):
            alpha[index] = (
                3.0 / intervals[index] * (values[index + 1] - values[index])
                - 3.0 / intervals[index - 1] * (values[index] - values[index - 1])
            )

        lower = [1.0] + [0.0] * (count - 1)
        upper = [0.0] * count
        rhs = [0.0] * count
        for index in range(1, count - 1):
            lower[index] = (
                2.0 * (knots[index + 1] - knots[index - 1])
                - intervals[index - 1] * upper[index - 1]
            )
            if lower[index] <= 0 or not math.isfinite(lower[index]):
                raise SmoothingError("spline system is numerically unstable")
            upper[index] = intervals[index] / lower[index]
            rhs[index] = (
                alpha[index] - intervals[index - 1] * rhs[index - 1]
            ) / lower[index]

        second_derivative = [0.0] * count
        for index in range(count - 2, -1, -1):
            second_derivative[index] = (
                rhs[index] - upper[index] * second_derivative[index + 1]
            )

        linear = [0.0] * (count - 1)
        cubic = [0.0] * (count - 1)
        for index, interval in enumerate(intervals):
            linear[index] = (
                (values[index + 1] - values[index]) / interval
                - interval
                * (second_derivative[index + 1] + 2.0 * second_derivative[index])
                / 3.0
            )
            cubic[index] = (
                second_derivative[index + 1] - second_derivative[index]
            ) / (3.0 * interval)
        return list(values[:-1]), linear, list(second_derivative[:-1]), cubic

    @staticmethod
    def _evaluate(
        knots: Sequence[float],
        coefficients: tuple[list[float], list[float], list[float], list[float]],
        value: float,
    ) -> float:
        constants, linear, quadratic, cubic = coefficients
        interval_index = len(constants) - 1
        for index in range(len(knots) - 1):
            if value <= knots[index + 1]:
                interval_index = index
                break
        offset = value - knots[interval_index]
        return (
            constants[interval_index]
            + linear[interval_index] * offset
            + quadratic[interval_index] * offset * offset
            + cubic[interval_index] * offset * offset * offset
        )
