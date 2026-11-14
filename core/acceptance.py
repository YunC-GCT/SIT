"""Repeatable end-to-end simulation acceptance for the modular workflow."""

from __future__ import annotations

import json
import os
from typing import Any, Sequence

from motion_control.src.simulation import (
    MotionSimulation,
    StopPathSimulation,
    simulate_exception_stop,
    simulate_keyboard_interrupt_stop,
    simulate_segments,
)

from .batch import RunBatch
from .contracts import (
    STATUS_FAILED,
    STATUS_SUCCESS,
    make_error,
    make_result,
    require_success,
    write_result,
)
from .orchestrator import run_pipeline
from .process_runner import PROJECT_ROOT


MODULE = "acceptance"
PIPELINE_STAGES = ("map_pipeline", "path_planner", "path_postprocess")


def _check(name: str, passed: bool, details: dict[str, Any]) -> dict[str, Any]:
    return {"name": name, "status": STATUS_SUCCESS if passed else STATUS_FAILED, **details}


def _manual_record_template() -> dict[str, Any]:
    return {
        "status": "pending_manual",
        "environment": {
            "host": "Raspberry Pi Debian 12",
            "container": "turbopi",
            "ros_distribution": "Humble",
            "project_path": "/home/ubuntu/SIT_LJGH_PROJECT",
        },
        "conditions": {
            "battery": "",
            "floor": "",
            "payload": "",
            "notes": "",
        },
        "direction_checks": [
            {"name": "forward", "command": {}, "actual_direction": "", "result": ""},
            {"name": "backward", "command": {}, "actual_direction": "", "result": ""},
            {"name": "left", "command": {}, "actual_direction": "", "result": ""},
            {"name": "right", "command": {}, "actual_direction": "", "result": ""},
            {"name": "ninety_degree_turn", "command": {}, "actual_direction": "", "result": ""},
        ],
        "path_following": {
            "result": "",
            "actual_direction": "",
            "empirical_speed_m_s": None,
            "cmd_vel_topic": "/cmd_vel",
            "odom_available": False,
            "notes": "",
        },
        "safety": {
            "other_cmd_vel_publishers_isolated": False,
            "chassis_subscriber": "/mecanum_chassis_node",
            "zero_twist_observed_on_exit": False,
            "ctrl_c_stop_observed": False,
        },
    }


def _stage_checks(batch: RunBatch) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    checks = []
    stage_details: dict[str, Any] = {}
    for module in PIPELINE_STAGES:
        stage = batch.manifest.get("stages", {}).get(module, {})
        result_path = stage.get("result")
        valid = stage.get("status") == STATUS_SUCCESS and bool(result_path)
        result_status = None
        logs: list[str] = []
        if valid:
            try:
                result = require_success(result_path, module=module)
                result_status = result["status"]
                logs = [os.path.abspath(path) for path in result.get("logs", [])]
                valid = bool(logs) and all(os.path.exists(path) for path in logs)
            except Exception as exc:
                stage_details[module] = {"error": str(exc)}
                valid = False
        stage_details[module] = {
            "status": stage.get("status"),
            "result": os.path.abspath(result_path) if result_path else None,
            "result_status": result_status,
            "logs": logs,
            "logs_exist": bool(logs) and all(os.path.exists(path) for path in logs),
        }
        checks.append(
            _check(
                f"{module}_contract_and_trace",
                valid,
                {"module": module, "details": stage_details[module]},
            )
        )
    return checks, stage_details


def _simulation_checks(simulation: MotionSimulation) -> list[dict[str, Any]]:
    commands = simulation.commands
    moving = [command for command in commands if command != (0.0, 0.0, 0.0)]
    has_forward = any(linear_x > 0 and linear_y == 0 and angular_z == 0 for linear_x, linear_y, angular_z in moving)
    only_turn_or_forward = all(
        (linear_y == 0 and (linear_x == 0 or angular_z == 0))
        for linear_x, linear_y, angular_z in moving
    )
    return [
        _check(
            "fake_cmd_vel_preflight_fixture",
            simulation.preflight.confirmed
            and simulation.preflight.chassis_subscriber_seen
            and not simulation.preflight.other_publishers,
            {
                "details": {
                    "topic": simulation.preflight.topic,
                    "subscribers": list(simulation.preflight.subscribers),
                    "publishers": list(simulation.preflight.publishers),
                    "other_publishers": list(simulation.preflight.other_publishers),
                }
            },
        ),
        _check(
            "turn_and_forward_sequence",
            simulation.report.completed and has_forward and only_turn_or_forward,
            {
                "details": {
                    "segment_count": simulation.report.segment_count,
                    "command_count": simulation.report.command_count,
                    "open_loop": simulation.report.open_loop,
                    "completed": simulation.report.completed,
                    "stopped": simulation.report.stopped,
                }
            },
        ),
        _check(
            "safe_stop_and_shutdown",
            simulation.report.stopped
            and bool(commands)
            and commands[-1] == (0.0, 0.0, 0.0)
            and simulation.node_destroyed,
            {"details": {"last_command": commands[-1] if commands else None}},
        ),
    ]


def _stop_path_check(name: str, simulation: StopPathSimulation, expected: str) -> dict[str, Any]:
    return _check(
        name,
        simulation.exception_type == expected
        and simulation.stopped
        and simulation.node_destroyed
        and simulation.zero_before_destroy,
        {"details": simulation.to_payload()},
    )


def _write_manual_record(batch: RunBatch) -> str:
    path = os.path.join(batch.path, "acceptance", "manual_record.json")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(_manual_record_template(), handle, ensure_ascii=False, indent=2)
        handle.write("\n")
    batch.record_artifact("manual_hardware_record", path)
    return path


def run_simulated_acceptance(
    image_path: str | None = None,
    *,
    start: tuple[int, int] = (2, 2),
    goal: tuple[int, int] = (37, 37),
    mode: str = "normal",
    algorithms: Sequence[str] | None = None,
    seed: int = 42,
    run_root: str = "runs",
    run_id: str | None = None,
    map_config: str | None = None,
    planner_config: str | None = None,
    postprocess_config: str | None = None,
) -> dict:
    """Run real map/planner/postprocess CLIs and simulated motion in one batch."""
    image = image_path or os.path.join(
        PROJECT_ROOT,
        "tests",
        "fixtures",
        "01_scattered.png",
    )
    effective_map_config = map_config or os.path.join(
        PROJECT_ROOT,
        "tools",
        "acceptance_map.yaml",
    )
    pipeline = run_pipeline(
        image,
        start=start,
        goal=goal,
        mode=mode,
        algorithms=algorithms,
        seed=seed,
        headless=True,
        drive=False,
        run_root=run_root,
        run_id=run_id,
        map_config=effective_map_config,
        planner_config=planner_config,
        postprocess_config=postprocess_config,
    )
    batch = RunBatch(root=run_root, run_id=pipeline.run_id).reload()
    manual_record = _write_manual_record(batch)
    checks, stage_details = _stage_checks(batch)
    simulation: MotionSimulation | None = None
    keyboard_interrupt_stop: StopPathSimulation | None = None
    exception_stop: StopPathSimulation | None = None
    errors: list[dict] = []

    if pipeline.status == STATUS_SUCCESS and all(
        item["status"] == STATUS_SUCCESS for item in checks
    ):
        try:
            postprocess_path = stage_details["path_postprocess"]["result"]
            postprocess = require_success(postprocess_path, module="path_postprocess")
            outputs = postprocess["outputs"]
            if "comparisons" in outputs:
                successful = [
                    item for item in outputs["comparisons"] if item.get("status") == STATUS_SUCCESS
                ]
                if not successful:
                    raise ValueError("path_postprocess has no successful comparison")
                segments = successful[0]["outputs"]["segments"]
            else:
                segments = outputs["segments"]
            simulation = simulate_segments(segments)
            checks.extend(_simulation_checks(simulation))
            interrupt_segments = [
                {"index": 0, "direction_deg": 90.0, "distance_m": 0.3}
            ]
            keyboard_interrupt_stop = simulate_keyboard_interrupt_stop(interrupt_segments)
            exception_stop = simulate_exception_stop(interrupt_segments)
            checks.append(
                _stop_path_check(
                    "ctrl_c_safe_stop",
                    keyboard_interrupt_stop,
                    "KeyboardInterrupt",
                )
            )
            checks.append(
                _stop_path_check(
                    "exception_safe_stop",
                    exception_stop,
                    "RuntimeError",
                )
            )
        except Exception as exc:
            errors.append(
                make_error(
                    "SIMULATION_ERROR",
                    module=MODULE,
                    stage="simulate_motion",
                    error_type=type(exc).__name__,
                    reason=str(exc),
                    input_ref=stage_details.get("path_postprocess", {}).get("result", "<missing>"),
                    suggestion="inspect the path_postprocess result and rerun acceptance",
                )
            )
    else:
        errors.extend(pipeline.errors)

    passed = not errors and all(item["status"] == STATUS_SUCCESS for item in checks)
    if not passed and not errors:
        errors.append(
            make_error(
                "ACCEPTANCE_CHECK_FAILED",
                module=MODULE,
                stage="verify",
                error_type="AcceptanceError",
                reason="one or more automated acceptance checks failed",
                input_ref=batch.manifest_path,
                suggestion="inspect acceptance/result.json and the preserved stage artifacts",
            )
        )

    output_path = os.path.join(batch.stage_dir(MODULE), "result.json")
    log_path = batch.log_path(MODULE)
    with open(log_path, "a", encoding="utf-8") as log:
        log.write(
            f"run_id={batch.run_id} mode={mode} status="
            f"{STATUS_SUCCESS if passed else STATUS_FAILED}\n"
        )
    result = make_result(
        MODULE,
        batch.run_id,
        STATUS_SUCCESS if passed else STATUS_FAILED,
        inputs={
            "image": os.path.abspath(image),
            "start": {"row": start[0], "col": start[1]},
            "goal": {"row": goal[0], "col": goal[1]},
        },
        outputs={
            "workflow": {
                "status": pipeline.status,
                "manifest": batch.manifest_path,
                "preview": pipeline.preview_path,
            },
            "stages": stage_details,
            "checks": checks,
            "simulation": simulation.to_payload() if simulation else None,
            "keyboard_interrupt_stop": (
                keyboard_interrupt_stop.to_payload()
                if keyboard_interrupt_stop
                else None
            ),
            "exception_stop": (
                exception_stop.to_payload()
                if exception_stop
                else None
            ),
            "manual_hardware_acceptance": {
                "status": "pending_manual",
                "record": manual_record,
                "note": "simulation evidence does not replace low-speed real-vehicle acceptance",
            },
        },
        parameters={
            "mode": mode,
            "algorithms": list(algorithms or []),
            "seed": seed,
            "map_config": os.path.abspath(effective_map_config),
        },
        errors=errors,
        logs=[log_path],
        metadata={"acceptance_scope": "pipeline_and_fake_ros2"},
    )
    write_result(output_path, result)
    batch.update_stage(
        MODULE,
        result["status"],
        result=output_path,
        error=result["errors"] if result["status"] == STATUS_FAILED else None,
    )
    batch.finish(result["status"])
    return result


__all__ = ["run_simulated_acceptance"]
