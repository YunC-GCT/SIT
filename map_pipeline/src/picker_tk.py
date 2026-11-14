"""Tk two-point picker."""

from __future__ import annotations

import os
from typing import List, Tuple

import numpy as np

from .coordinate_mapper import map_pixel_to_grid
from .grid_map import GridMap
from .picker_common import (
    PointSelectionCancelled,
    _marker_radius,
    _picker_image,
    _snap_to_grid_center,
    _to_rgb,
    _write_temp_ppm,
)

def _pick_points_tk(
    img: np.ndarray,
    window_name: str,
    grid_map: GridMap | None,
) -> Tuple[Tuple[int, int], Tuple[int, int]]:
    """Use Tk to avoid Windows OpenCV HighGUI display offset issues."""
    import tkinter as tk

    height, width = img.shape[:2]
    if height < 1 or width < 1:
        raise ValueError("选点图像不能为空")

    base_display, scale, display_width, display_height = _picker_image(img)
    temp_path = _write_temp_ppm(_to_rgb(base_display))
    points: List[Tuple[int, int]] = []
    message = "Select start, then goal (Enter=confirm, R=reset, Esc=cancel)"
    state = {"cancelled": False, "reason": "已取消起终点选择"}

    try:
        root = tk.Tk()
    except tk.TclError as exc:
        try:
            os.unlink(temp_path)
        except OSError:
            pass
        raise RuntimeError("Tk window backend is unavailable") from exc

    root.title(window_name)
    root.resizable(False, False)
    try:
        photo = tk.PhotoImage(file=temp_path)
    except tk.TclError as exc:
        root.destroy()
        try:
            os.unlink(temp_path)
        except OSError:
            pass
        raise RuntimeError("Tk could not load the picker image") from exc
    canvas = tk.Canvas(
        root,
        width=display_width,
        height=display_height,
        highlightthickness=0,
        takefocus=True,
    )
    canvas.pack()
    canvas.create_image(0, 0, image=photo, anchor=tk.NW)

    def to_original_point(x: int, y: int) -> Tuple[int, int]:
        original_x = max(0, min(width - 1, int(x / scale)))
        original_y = max(0, min(height - 1, int(y / scale)))
        return original_x, original_y

    def point_info(point: Tuple[int, int]) -> Tuple[Tuple[int, int] | None, str]:
        if grid_map is None:
            return None, "grid=unknown"
        grid_point = map_pixel_to_grid(point[0], point[1], grid_map)
        state_name = "FREE" if grid_map.is_free(*grid_point) else "OBSTACLE"
        return grid_point, state_name

    def draw() -> None:
        canvas.delete("overlay")
        if grid_map is not None:
            for col in range(1, grid_map.cols):
                x = int(col * grid_map.cell_width * scale)
                canvas.create_line(
                    x,
                    0,
                    x,
                    display_height,
                    fill="#b4b4b4",
                    tags="overlay",
                )
            for row in range(1, grid_map.rows):
                y = int(row * grid_map.cell_height * scale)
                canvas.create_line(
                    0,
                    y,
                    display_width,
                    y,
                    fill="#b4b4b4",
                    tags="overlay",
                )

        for index, point in enumerate(points):
            x = int(point[0] * scale)
            y = int(point[1] * scale)
            color = "#00cc00" if index == 0 else "#cc0000"
            label = "S" if index == 0 else "G"
            radius = _marker_radius(scale, grid_map)
            canvas.create_oval(
                x - radius,
                y - radius,
                x + radius,
                y + radius,
                outline=color,
                width=2,
                tags="overlay",
            )
            canvas.create_text(
                x + 10,
                y - 8,
                text=label,
                anchor=tk.W,
                fill=color,
                font=("TkDefaultFont", 12, "bold"),
                tags="overlay",
            )
            grid_point, state_name = point_info(point)
            detail = (
                f"{label}: pixel=({point[0]},{point[1]}) {state_name}"
                if grid_point is None
                else f"{label}: grid=({grid_point[0]},{grid_point[1]}) {state_name}"
            )
            canvas.create_text(
                10,
                22 + index * 22,
                text=detail,
                anchor=tk.W,
                fill=color,
                font=("TkDefaultFont", 10, "bold"),
                tags="overlay",
            )

        canvas.create_rectangle(
            0,
            display_height - 28,
            display_width,
            display_height,
            fill="#000000",
            outline="",
            tags="overlay",
        )
        canvas.create_text(
            10,
            display_height - 14,
            text=message,
            anchor=tk.W,
            fill="#ffffff",
            font=("TkDefaultFont", 10),
            tags="overlay",
        )

    def update_message(value: str) -> None:
        nonlocal message
        message = value
        draw()

    def on_left(event: object) -> None:
        x = int(getattr(event, "x"))
        y = int(getattr(event, "y"))
        if len(points) >= 2:
            update_message("Two points selected; use R or right-click to change")
            return
        points.append(_snap_to_grid_center(to_original_point(x, y), grid_map))
        _, state_name = point_info(points[-1])
        update_message(
            "Selected obstacle; choose another point"
            if state_name == "OBSTACLE"
            else "Select the goal (right-click=undo)"
            if len(points) == 1
            else "Press Enter to confirm (R=reset, right-click=undo)"
        )

    def on_right(_event: object | None = None) -> None:
        if points:
            points.pop()
            update_message("Last point removed")
        else:
            update_message("No point to remove")

    def cancel(reason: str = "已取消起终点选择") -> None:
        state["cancelled"] = True
        state["reason"] = reason
        root.quit()

    def confirm(_event: object | None = None) -> None:
        if len(points) != 2:
            update_message("Enter requires start and goal")
            return
        if grid_map is not None and any(
            not grid_map.is_free(*map_pixel_to_grid(x, y, grid_map))
            for x, y in points
        ):
            update_message("Start and goal must be on free cells")
            return
        root.quit()

    def reset(_event: object | None = None) -> None:
        points.clear()
        update_message("Selection reset")

    canvas.bind("<Button-1>", on_left)
    canvas.bind("<Button-3>", on_right)
    root.bind_all("<Return>", confirm)
    root.bind_all("<KP_Enter>", confirm)
    root.bind_all("<Escape>", lambda _event: cancel())
    root.bind_all("r", reset)
    root.bind_all("R", reset)
    root.protocol(
        "WM_DELETE_WINDOW",
        lambda: cancel("选点窗口被关闭，已取消起终点选择"),
    )

    try:
        draw()
        root.update_idletasks()
        root.focus_force()
        canvas.focus_set()
        root.mainloop()
    finally:
        root.destroy()
        try:
            os.unlink(temp_path)
        except OSError:
            pass

    if state["cancelled"]:
        raise PointSelectionCancelled(str(state["reason"]))
    if len(points) != 2:
        raise PointSelectionCancelled("起终点未选全，已取消选择")
    return points[0], points[1]


__all__ = ["_pick_points_tk"]
