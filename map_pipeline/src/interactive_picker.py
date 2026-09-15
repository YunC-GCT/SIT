"""
在原图上交互点击选取起点和终点，返回像素坐标。

选点的公开边界只有两个点：起点和终点。栅格地图作为可选输入时，
窗口会同时显示映射后的 row/col 和空闲/障碍状态。
"""

from __future__ import annotations

from typing import List, Tuple

import cv2
import numpy as np

from .coordinate_mapper import map_pixel_to_grid
from .grid_map import GridMap


class PointSelectionCancelled(RuntimeError):
    """用户取消了起终点选择，或提前关闭了选点窗口。"""


def _display_geometry(
    width: int,
    height: int,
    max_width: int = 1000,
    max_height: int = 700,
) -> tuple[float, int, int]:
    """Return a display scale and dimensions that use the available canvas."""
    if width < 1 or height < 1:
        raise ValueError("显示图像尺寸必须为正数")
    if max_width < 1 or max_height < 1:
        raise ValueError("显示窗口尺寸必须为正数")

    scale = min(max_width / width, max_height / height)
    return scale, max(1, int(round(width * scale))), max(
        1, int(round(height * scale))
    )


def _normalize_key(key: object) -> int:
    """Normalize OpenCV backend key codes to the low-byte key value."""
    try:
        value = int(key)
    except (TypeError, ValueError):
        return -1
    return value if value < 0 else value & 0xFF


def _is_confirm_key(key: object) -> bool:
    """Recognize Enter/Return codes emitted by common OpenCV backends."""
    try:
        value = int(key)
    except (TypeError, ValueError):
        return False
    if _normalize_key(value) in (10, 13):
        return True
    return value in {
        0x01000004,  # Qt::Key_Return
        0x01000005,  # Qt::Key_Enter
    }


def pick_points(
    image: np.ndarray,
    window_name: str = "Select Start & Goal",
    grid_map: GridMap | None = None,
) -> Tuple[Tuple[int, int], Tuple[int, int]]:
    """
    通过一次起点和一次终点点击选择像素坐标。

    Enter 仅在两点都已选择且位于空闲栅格时确认；R 清空，右键撤销最近
    一次点击，Esc 或关闭窗口取消。障碍栅格不会被自动替换。
    """
    if not isinstance(image, np.ndarray) or image.ndim < 2:
        raise ValueError("选点图像必须是至少二维的 numpy 数组")

    img = image.copy()
    height, width = img.shape[:2]
    if height < 1 or width < 1:
        raise ValueError("选点图像不能为空")

    scale, display_width, display_height = _display_geometry(width, height)
    interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_NEAREST
    base_display = cv2.resize(
        img,
        (display_width, display_height),
        interpolation=interpolation,
    )
    points: List[Tuple[int, int]] = []
    message = "Select start, then goal (Enter=confirm, R=reset, Esc=cancel)"

    def point_info(point: Tuple[int, int]) -> Tuple[Tuple[int, int] | None, str]:
        if grid_map is None:
            return None, "grid=unknown"
        grid_point = map_pixel_to_grid(point[0], point[1], grid_map)
        state = "FREE" if grid_map.is_free(*grid_point) else "OBSTACLE"
        return grid_point, state

    def draw() -> None:
        output = base_display.copy()

        if grid_map is not None:
            for col in range(1, grid_map.cols):
                x = int(col * grid_map.cell_width * scale)
                cv2.line(output, (x, 0), (x, display_height), (180, 180, 180), 1)
            for row in range(1, grid_map.rows):
                y = int(row * grid_map.cell_height * scale)
                cv2.line(output, (0, y), (display_width, y), (180, 180, 180), 1)

        for index, point in enumerate(points):
            display_point = (
                int(point[0] * scale),
                int(point[1] * scale),
            )
            color = (0, 255, 0) if index == 0 else (0, 0, 255)
            label = "S" if index == 0 else "G"
            marker_radius = max(5, min(18, int(round(9 * scale))))
            cv2.circle(output, display_point, marker_radius, color, 2)
            cv2.putText(
                output,
                label,
                (display_point[0] + 10, display_point[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2,
            )
            grid_point, state = point_info(point)
            if grid_point is None:
                detail = f"{label}: pixel=({point[0]},{point[1]}) {state}"
            else:
                detail = (
                    f"{label}: grid=({grid_point[0]},{grid_point[1]}) {state}"
                )
            cv2.putText(
                output,
                detail,
                (10, 25 + index * 22),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                color,
                1,
            )

        cv2.putText(
            output,
            message,
            (10, max(20, display_height - 12)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (255, 255, 255),
            1,
        )
        cv2.imshow(window_name, output)

    def to_original_point(x: int, y: int) -> Tuple[int, int]:
        original_x = max(0, min(width - 1, int(x / scale)))
        original_y = max(0, min(height - 1, int(y / scale)))
        return original_x, original_y

    def on_mouse(event: int, x: int, y: int, _flags: int, _param: object) -> None:
        nonlocal message
        if event == cv2.EVENT_LBUTTONDOWN:
            if len(points) >= 2:
                message = "Two points selected; use R or right-click to change"
            else:
                points.append(to_original_point(x, y))
                _, state = point_info(points[-1])
                message = (
                    "Selected obstacle; choose another point"
                    if state == "OBSTACLE"
                    else "Select the goal (right-click=undo)"
                    if len(points) == 1
                    else "Press Enter to confirm (R=reset, right-click=undo)"
                )
            draw()
        elif event == cv2.EVENT_RBUTTONDOWN:
            if points:
                points.pop()
                message = "Last point removed"
            else:
                message = "No point to remove"
            draw()

    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(window_name, on_mouse)
    draw()

    cancelled = False
    cancel_reason = "已取消起终点选择"
    try:
        while True:
            wait_key_ex = getattr(cv2, "waitKeyEx", None)
            raw_key = (
                wait_key_ex(50)
                if callable(wait_key_ex)
                else cv2.waitKey(50)
            )
            key = _normalize_key(raw_key)
            try:
                visible = cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE)
            except cv2.error:
                visible = -1
            if visible < 1:
                cancelled = True
                cancel_reason = "选点窗口被关闭，已取消起终点选择"
                break

            if key in (ord("r"), ord("R")):
                points.clear()
                message = "Selection reset"
                draw()
            elif _is_confirm_key(raw_key):
                if len(points) != 2:
                    message = "Enter requires start and goal"
                    draw()
                elif grid_map is not None and any(
                    not grid_map.is_free(*map_pixel_to_grid(x, y, grid_map))
                    for x, y in points
                ):
                    message = "Start and goal must be on free cells"
                    draw()
                else:
                    break
            elif key == 27:
                cancelled = True
                break
    finally:
        cv2.destroyWindow(window_name)

    if cancelled:
        raise PointSelectionCancelled(cancel_reason)
    if len(points) != 2:
        raise PointSelectionCancelled("起终点未选全，已取消选择")

    return points[0], points[1]
