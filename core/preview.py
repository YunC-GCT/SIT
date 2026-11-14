"""顶层工作流使用的路径预览渲染。"""

from __future__ import annotations

import os

import cv2
import numpy as np

from .contracts import STATUS_SUCCESS, require_success


def _destroy_window_safely(window_name: str) -> None:
    """关闭 OpenCV 窗口，容忍用户已经关闭窗口造成的后端异常。"""
    try:
        visible = cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE)
    except cv2.error:
        return
    if visible < 1:
        return
    try:
        cv2.destroyWindow(window_name)
    except cv2.error:
        pass


def _as_path(points_payload: dict) -> list[tuple[float, float]]:
    """将公共路径载荷中的点数组转换为用于绘图的行列坐标序列。"""
    values = points_payload.get("points", {}).get("values", [])
    return [(float(point[0]), float(point[1])) for point in values]


def save_path_preview(
    map_result_path: str,
    planner_result_path: str,
    postprocess_result_path: str,
    output_path: str,
) -> str:
    """读取地图、规划和后处理的公共 JSON，绘制路径预览并返回保存路径。"""
    map_result = require_success(map_result_path, module="map_pipeline")
    planner_result = require_success(planner_result_path, module="path_planner")
    postprocess_result = require_success(
        postprocess_result_path,
        module="path_postprocess",
    )
    grid_payload = map_result["outputs"]["map"]
    grid = np.asarray(grid_payload["grid"]["values"], dtype=np.uint8)
    start_payload = grid_payload["start"]
    goal_payload = grid_payload["goal"]
    start = (int(start_payload["row"]), int(start_payload["col"]))
    goal = (int(goal_payload["row"]), int(goal_payload["col"]))

    from path_planner.preview import draw_comparison, draw_path

    comparisons = planner_result["outputs"].get("comparisons")
    if isinstance(comparisons, list):
        comparison_data = {}
        for item in comparisons:
            raw_path = item.get("raw_path", {})
            comparison_data[item.get("algorithm", "planner")] = {
                "path": [
                    (int(point[0]), int(point[1]))
                    for point in raw_path.get("points", {}).get("values", [])
                ],
                "time": item.get("metrics", {}).get("values", {}).get(
                    "planning_time_s", 0.0
                ),
            }
        image = draw_comparison(grid, comparison_data, start, goal)
    else:
        raw_path = planner_result["outputs"].get("raw_path", {})
        image = draw_path(
            grid,
            [
                (int(point[0]), int(point[1]))
                for point in raw_path.get("points", {}).get("values", [])
            ],
            start,
            goal,
        )

    effective_cell = max(1, image.shape[1] // max(grid.shape[1], 1))
    smoothed = _as_path(postprocess_result["outputs"].get("smoothed_path", {}))
    if len(smoothed) >= 2:
        points = np.asarray(
            [
                [
                    int(round(col * effective_cell + effective_cell / 2)),
                    int(round(row * effective_cell + effective_cell / 2)),
                ]
                for row, col in smoothed
            ],
            dtype=np.int32,
        )
        cv2.polylines(image, [points], False, (0, 128, 255), 2)

    segments = postprocess_result["outputs"].get("segments", [])
    postprocess_comparisons = postprocess_result["outputs"].get("comparisons")
    if not segments and isinstance(postprocess_comparisons, list):
        segments = [
            segment
            for comparison in postprocess_comparisons
            for segment in comparison.get("outputs", {}).get("segments", [])
        ]
    for segment in segments:
        begin = segment["start"]
        end = segment["end"]
        p1 = (
            int(begin["col"] * effective_cell + effective_cell / 2),
            int(begin["row"] * effective_cell + effective_cell / 2),
        )
        p2 = (
            int(end["col"] * effective_cell + effective_cell / 2),
            int(end["row"] * effective_cell + effective_cell / 2),
        )
        cv2.line(image, p1, p2, (0, 165, 0), 1)

    postprocess_outputs = postprocess_result["outputs"]
    if "comparisons" in postprocess_outputs:
        successful = [
            item
            for item in postprocess_outputs["comparisons"]
            if item.get("status") == STATUS_SUCCESS
        ]
        if not successful:
            raise ValueError("postprocess result contains no successful comparison")
        postprocess_outputs = successful[0]["outputs"]
        segments = postprocess_outputs.get("segments", [])
    metrics = postprocess_outputs.get("metrics", {}).get("values", {})
    estimated = metrics.get(
        "estimated_time_s",
        metrics.get("estimated_execution_time_s", 0.0),
    )
    cv2.rectangle(image, (0, 0), (min(image.shape[1], 620), 30), (0, 0, 0), -1)
    cv2.putText(
        image,
        f"segments={len(segments)} estimated={float(estimated):.2f}s open-loop",
        (8, 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    output_path = os.path.abspath(output_path)
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    extension = os.path.splitext(output_path)[1] or ".png"
    encoded_ok, encoded = cv2.imencode(extension, image)
    if not encoded_ok:
        raise OSError(f"unable to encode path preview: {output_path}")
    try:
        with open(output_path, "wb") as handle:
            handle.write(encoded.tobytes())
    except OSError as exc:
        raise OSError(f"unable to write path preview: {output_path}") from exc
    return os.path.abspath(output_path)


__all__ = ["_destroy_window_safely", "save_path_preview"]
