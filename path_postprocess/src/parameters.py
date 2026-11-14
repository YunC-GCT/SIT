"""Path post-processing parameter validation."""

from __future__ import annotations

import math
from numbers import Real
from typing import Any

DEFAULT_PARAMETERS = {
    "smooth": True,
    "sample_spacing_grid": 0.25,
    "max_segment_length_grid": 2.0,
    "move_speed_m_s": 0.30,
    "turn_speed_rad_s": 2.0,
    "initial_heading": "north",
}

HEADINGS_DEG = {
    "north": 0.0,
    "east": 90.0,
    "south": 180.0,
    "west": 270.0,
}

class PathPostprocessParameterError(ValueError):
    """Raised when post-processing parameters are invalid."""

def resolve_postprocess_parameters(
    parameters: dict[str, Any] | None = None,
    *,
    grid_size_m: float = 0.3,
) -> dict[str, Any]:
    """Validate and return effective post-processing parameters."""
    if parameters is not None and not isinstance(parameters, dict):
        raise PathPostprocessParameterError("parameters must be a JSON object")
    values = dict(DEFAULT_PARAMETERS)
    provided = dict(parameters or {})
    aliases = {
        "sample_spacing_m",
        "max_segment_length_m",
    }
    unknown = sorted(set(provided) - set(values) - aliases)
    if unknown:
        raise PathPostprocessParameterError(
            f"unsupported post-processing parameters: {', '.join(unknown)}"
        )
    if (
        not isinstance(grid_size_m, Real)
        or isinstance(grid_size_m, bool)
        or not math.isfinite(float(grid_size_m))
        or grid_size_m <= 0
    ):
        raise PathPostprocessParameterError("grid_size_m must be positive")

    if "sample_spacing_m" in provided and "sample_spacing_grid" not in provided:
        try:
            provided["sample_spacing_grid"] = (
                float(provided["sample_spacing_m"]) / float(grid_size_m)
            )
        except (TypeError, ValueError, ZeroDivisionError) as exc:
            raise PathPostprocessParameterError(
                "sample_spacing_m must be a positive number"
            ) from exc
    if (
        "max_segment_length_m" in provided
        and "max_segment_length_grid" not in provided
    ):
        try:
            provided["max_segment_length_grid"] = (
                float(provided["max_segment_length_m"]) / float(grid_size_m)
            )
        except (TypeError, ValueError, ZeroDivisionError) as exc:
            raise PathPostprocessParameterError(
                "max_segment_length_m must be a positive number"
            ) from exc
    values.update(provided)

    if not isinstance(values["smooth"], bool):
        raise PathPostprocessParameterError("smooth must be boolean")
    for name in (
        "sample_spacing_grid",
        "max_segment_length_grid",
        "move_speed_m_s",
        "turn_speed_rad_s",
    ):
        value = values[name]
        if (
            not isinstance(value, Real)
            or isinstance(value, bool)
            or not math.isfinite(float(value))
            or value <= 0
        ):
            raise PathPostprocessParameterError(f"{name} must be positive")
        values[name] = float(value)
    heading = values["initial_heading"]
    if values["max_segment_length_grid"] < 1.0:
        raise PathPostprocessParameterError(
            "max_segment_length_grid must be at least one grid cell"
        )
    if isinstance(heading, str):
        if heading.lower() not in HEADINGS_DEG:
            raise PathPostprocessParameterError(
                "initial_heading must be north, east, south, or west"
            )
        values["initial_heading"] = heading.lower()
    elif isinstance(heading, Real) and not isinstance(heading, bool):
        if not math.isfinite(float(heading)):
            raise PathPostprocessParameterError("initial_heading must be finite")
        values["initial_heading"] = float(heading) % 360.0
    else:
        raise PathPostprocessParameterError(
            "initial_heading must be a compass name or degree value"
        )
    return values


__all__ = [
    "DEFAULT_PARAMETERS",
    "HEADINGS_DEG",
    "PathPostprocessParameterError",
    "resolve_postprocess_parameters",
]
