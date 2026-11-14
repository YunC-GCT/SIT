"""OpenCV four-corner picker used for perspective rectification."""

from __future__ import annotations

from typing import List, Tuple

import cv2
import numpy as np

from .picker_common import (
    PointSelectionCancelled,
    _is_confirm_key,
    _normalize_key,
    _picker_image,
)

def pick_quad(
    image: np.ndarray,
    window_name: str = "Select Arena Corners",
) -> list[Tuple[int, int]]:
    """Click four arena corners in the image, in any consistent order."""
    if not isinstance(image, np.ndarray) or image.ndim < 2:
        raise ValueError("选点图像必须是至少二维的 numpy 数组")
    base_display, scale, display_width, display_height = _picker_image(image)
    height, width = image.shape[:2]
    points: List[Tuple[int, int]] = []
    message = (
        "Click 4 arena corners (Enter=confirm, R=reset, "
        "right-click=undo, Esc=cancel)"
    )

    def to_original_point(x: int, y: int) -> Tuple[int, int]:
        original_x = max(0, min(width - 1, int(x / scale)))
        original_y = max(0, min(height - 1, int(y / scale)))
        return original_x, original_y

    def draw() -> None:
        output = base_display.copy()
        for index, point in enumerate(points):
            display_point = (int(point[0] * scale), int(point[1] * scale))
            cv2.circle(output, display_point, 5, (0, 255, 0), -1)
            cv2.putText(
                output,
                str(index + 1),
                (display_point[0] + 8, display_point[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )
        if len(points) > 1:
            polygon = np.asarray(
                [(int(x * scale), int(y * scale)) for x, y in points],
                dtype=np.int32,
            )
            cv2.polylines(output, [polygon], False, (0, 160, 255), 2)
        if len(points) == 4:
            cv2.line(
                output,
                (int(points[3][0] * scale), int(points[3][1] * scale)),
                (int(points[0][0] * scale), int(points[0][1] * scale)),
                (0, 160, 255),
                2,
            )
        cv2.putText(
            output,
            message,
            (10, max(20, display_height - 12)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
        )
        cv2.imshow(window_name, output)

    def on_mouse(event: int, x: int, y: int, _flags: int, _param: object) -> None:
        nonlocal message
        if event == cv2.EVENT_LBUTTONDOWN:
            if len(points) >= 4:
                message = "Four corners selected; Enter=confirm, R=reset"
            else:
                points.append(to_original_point(x, y))
                message = (
                    f"{len(points)}/4 corners selected"
                    if len(points) < 4
                    else "Press Enter to confirm (R=reset, right-click=undo)"
                )
            draw()
        elif event == cv2.EVENT_RBUTTONDOWN:
            if points:
                points.pop()
                message = "Last corner removed"
            else:
                message = "No corner to remove"
            draw()

    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, display_width, display_height)
    cv2.setMouseCallback(window_name, on_mouse)
    draw()
    cv2.waitKey(1)

    cancelled = False
    cancel_reason = "已取消四角选择"
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
                cancel_reason = "四角选择窗口被关闭，已取消矫正"
                break
            if key in (ord("r"), ord("R")):
                points.clear()
                message = "Selection reset"
                draw()
            elif _is_confirm_key(raw_key):
                if len(points) == 4:
                    break
                message = "Enter requires four corners"
                draw()
            elif key == 27:
                cancelled = True
                break
    finally:
        cv2.destroyWindow(window_name)

    if cancelled:
        raise PointSelectionCancelled(cancel_reason)
    if len(points) != 4:
        raise PointSelectionCancelled("四角未选全，已取消矫正")
    return points


__all__ = ["pick_quad"]
