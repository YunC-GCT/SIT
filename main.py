#!/usr/bin/env python3
"""Top-level normal/research workflow entry point."""

from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Sequence

from core import display_available, interactive_terminal
from core.orchestrator import PipelineReport, run_pipeline


def _pair(value: str) -> tuple[int, int]:
    try:
        row, col = value.split(",", 1)
        return int(row), int(col)
    except (AttributeError, TypeError, ValueError) as exc:
        raise argparse.ArgumentTypeError("coordinate must use row,col") from exc


def _json_object(value: str | None, option: str) -> dict | None:
    if value is None:
        return None
    try:
        parsed = json.loads(value)
    except json.JSONDecodeError as exc:
        raise argparse.ArgumentTypeError(f"{option} must be a JSON object") from exc
    if not isinstance(parsed, dict):
        raise argparse.ArgumentTypeError(f"{option} must be a JSON object")
    return parsed


def _choose_image(requested: str | None, *, interactive: bool) -> str | None:
    if requested:
        return requested
    if not interactive:
        return None
    input_dir = os.path.join(os.path.dirname(__file__), "map_pipeline", "data", "input")
    if not os.path.isdir(input_dir):
        return None
    extensions = (".jpg", ".jpeg", ".png", ".bmp")
    files = [
        name
        for name in sorted(os.listdir(input_dir))
        if name.lower().endswith(extensions)
    ]
    if not files:
        return None
    if interactive:
        print("Available maps:")
        for index, name in enumerate(files, 1):
            print(f"  [{index}] {name}")
        choice = input("Map number (blank uses first): ").strip()
        if choice:
            try:
                return os.path.join(input_dir, files[int(choice) - 1])
            except (ValueError, IndexError) as exc:
                raise ValueError("invalid map number") from exc
    return os.path.join(input_dir, files[0])


def run_script(script_path: str) -> bool:
    """Compatibility wrapper for callers that still launch one module script."""
    from core.orchestrator import _subprocess_runner

    return _subprocess_runner([sys.executable, script_path]) == 0


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run the modular path-planning and TurboPi workflow"
    )
    parser.add_argument("image", nargs="?", help="map image path")
    parser.add_argument("--image", dest="image_option", help="map image path")
    parser.add_argument("--mode", choices=("normal", "research"), default="normal")
    parser.add_argument("--research", action="store_true", help="alias for --mode research")
    parser.add_argument("--start", type=_pair, help="grid start row,col")
    parser.add_argument("--goal", type=_pair, help="grid goal row,col")
    parser.add_argument("--algorithm")
    parser.add_argument("--algorithms", help="comma-separated algorithm names")
    parser.add_argument("--params", type=lambda value: _json_object(value, "--params"))
    parser.add_argument(
        "--postprocess-params",
        type=lambda value: _json_object(value, "--postprocess-params"),
    )
    parser.add_argument("--seed", type=int)
    parser.add_argument(
        "--headless",
        action="store_true",
        help="run without GUI selection or preview windows",
    )
    parser.add_argument(
        "--select-terminal",
        action="store_true",
        help="prompt for start/goal row,col in the terminal instead of a GUI",
    )
    parser.add_argument(
        "--rectify",
        action="store_true",
        help="perspective-rectify the map image before gridding",
    )
    parser.add_argument(
        "--corners",
        help="four source corners x,y;x,y;x,y;x,y for rectification",
    )
    parser.add_argument(
        "--arena-width-m",
        type=float,
        help="real arena width in meters for grid scale calibration",
    )
    parser.add_argument(
        "--arena-height-m",
        type=float,
        help="real arena height in meters for rectified aspect ratio",
    )
    parser.add_argument(
        "--rectify-output-width",
        type=int,
        help="rectified image width in pixels",
    )
    parser.add_argument(
        "--grid-size-m",
        type=float,
        help="physical grid cell size in meters",
    )
    parser.add_argument("--drive", dest="drive", action="store_true")
    parser.add_argument("--no-drive", dest="drive", action="store_false")
    parser.set_defaults(drive=None)
    parser.add_argument(
        "--accept-path",
        action="store_true",
        help="accept the saved path preview in headless mode",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="confirm the final /cmd_vel motion start",
    )
    parser.add_argument("--run-root", default="runs")
    parser.add_argument("--run-id")
    parser.add_argument("--map-config")
    parser.add_argument("--planner-config")
    parser.add_argument("--postprocess-config")
    parser.add_argument("--motion-config")
    args = parser.parse_args(argv)

    headless = bool(args.headless or not interactive_terminal())
    terminal_select = bool(args.select_terminal)
    if terminal_select and not interactive_terminal():
        parser.error("--select-terminal requires interactive stdin and stdout")
    if not terminal_select and not headless and not display_available():
        terminal_select = True
    mode = "research" if args.research else args.mode
    if (args.start is None) != (args.goal is None):
        parser.error("--start and --goal must be provided together")

    try:
        image = _choose_image(
            args.image_option or args.image,
            interactive=not headless,
        )
        algorithms = (
            [name.strip() for name in args.algorithms.split(",") if name.strip()]
            if args.algorithms
            else None
        )
        report = run_pipeline(
            image,
            start=args.start,
            goal=args.goal,
            mode=mode,
            algorithm=args.algorithm,
            algorithms=algorithms,
            parameters=args.params,
            postprocess_parameters=args.postprocess_params,
            seed=args.seed,
            headless=headless,
            terminal_select=terminal_select,
            rectify=args.rectify or bool(args.corners),
            corners=args.corners,
            arena_width_m=args.arena_width_m,
            arena_height_m=args.arena_height_m,
            rectify_output_width=args.rectify_output_width,
            grid_size_m=args.grid_size_m,
            drive=args.drive,
            accept_path=args.accept_path,
            confirm_drive=args.yes,
            run_root=args.run_root,
            run_id=args.run_id,
            map_config=args.map_config,
            planner_config=args.planner_config,
            postprocess_config=args.postprocess_config,
            motion_config=args.motion_config,
            interactive_menu=not headless,
        )
    except (argparse.ArgumentTypeError, ValueError, OSError, TypeError) as exc:
        parser.error(str(exc))

    _print_report(report)
    return report.exit_code


def _print_report(report: PipelineReport) -> None:
    print(f"Workflow {report.status}: {report.manifest_path}")
    if report.preview_path:
        print(f"Path preview: {report.preview_path}")
    for error in report.errors:
        print(
            f"[{error.get('module', 'top_level')}] "
            f"{error.get('stage', 'unknown')}: "
            f"input={error.get('input', '<unknown>')} "
            f"{error.get('reason', 'unknown error')}; "
            f"suggestion={error.get('suggestion', 'retry')}",
            file=sys.stderr,
        )


if __name__ == "__main__":
    raise SystemExit(main())
