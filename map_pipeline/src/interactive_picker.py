"""
在原图上交互点击选取起点和终点，返回像素坐标
"""

import cv2
import numpy as np
from typing import Tuple, List, Optional

def pick_points(image: np.ndarray, window_name: str = "Select Start & Goal") -> Tuple[Tuple[int, int], Tuple[int, int]]:
    """
    显示原图，用户依次点击起点(绿)和终点(红)，按任意键确认。
    :returns: (start_pixel, goal_pixel) 每个像素坐标为 (x, y) 对应 (col, row)
    """
    img = image.copy()
    h, w = img.shape[:2]
    max_w, max_h = 1000, 700
    scale = min(max_w / w, max_h / h, 1.0)
    disp_w, disp_h = int(w * scale), int(h * scale)
    display_img = cv2.resize(img, (disp_w, disp_h))

    clicks: List[Tuple[int, int]] = []  # 实际像素坐标

    def _draw():
        out = display_img.copy()
        for i, (px, py) in enumerate(clicks):
            sx, sy = int(px * scale), int(py * scale)
            r = max(6, int(10 * scale))
            if i == 0:
                cv2.circle(out, (sx, sy), r, (0, 255, 0), 3)
                cv2.putText(out, "S", (sx + 15, sy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                cv2.circle(out, (sx, sy), r, (0, 0, 255), 3)
                cv2.putText(out, "G", (sx + 15, sy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.putText(out, "1st=START(Green) 2nd=GOAL(Red) | Any key to confirm | R=Reset",
                    (10, disp_h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.imshow(window_name, out)

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # 转换回原始尺寸坐标
            orig_x = int(x / scale)
            orig_y = int(y / scale)
            orig_x = max(0, min(w - 1, orig_x))
            orig_y = max(0, min(h - 1, orig_y))
            clicks.append((orig_x, orig_y))
            _draw()

    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_EXPANDED)
    cv2.resizeWindow(window_name, disp_w, disp_h)
    _draw()
    cv2.setMouseCallback(window_name, on_mouse)

    print("\n请点击起点，再点击终点，按任意键确认，或按 R 重置。")
    while True:
        key = cv2.waitKey(50) & 0xFF
        try:
            visible = cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE)
        except cv2.error:
            visible = -1
        if visible < 1:  # 窗口被关闭
            print("窗口已关闭，使用默认点（图片左上角和右下角）。")
            clicks = [(0, 0), (w-1, h-1)]
            break
        if key in (ord('r'), ord('R')):
            clicks.clear()
            _draw()
        elif key != 255:
            break

    cv2.destroyWindow(window_name)
    if len(clicks) < 2:
        clicks = [(0, 0), (w-1, h-1)]
    start_pixel = clicks[0]
    goal_pixel = clicks[1]
    print(f"已选择起点像素: {start_pixel}, 终点像素: {goal_pixel}")
    return start_pixel, goal_pixel