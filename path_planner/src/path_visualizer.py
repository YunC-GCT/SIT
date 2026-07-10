"""
路径可视化 — 纯 OpenCV 实现，cv2.imshow 弹窗预览。
无 matplotlib 依赖，树莓派/Docker 无头环境通过 X11 转发或 cv2.headless 降级。
"""
import numpy as np
import cv2

# 颜色常量 (BGR)
CLR_PATH = (255, 255, 0)   # 青色
CLR_START = (0, 255, 0)    # 绿
CLR_GOAL = (0, 0, 255)     # 红
CLR_GRID = (200, 200, 200) # 浅灰栅格线
CLR_FREE = 255             # 白（空闲格）


def _render(grid, max_w=1200, max_h=800, cell_px=16):
    """公共渲染：放大栅格为 BGR 图。返回 (bgr_image, effective_cell_px)。"""
    h, w = grid.shape[:2]
    eff = min(cell_px, max_w // max(w, 1), max_h // max(h, 1))
    eff = max(eff, 4)
    cw, ch = w * eff, h * eff

    img = np.where(grid == 1, 0, CLR_FREE).astype(np.uint8)
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    if eff != 1:
        img = cv2.resize(img, (cw, ch), interpolation=cv2.INTER_NEAREST)

    # 栅格线
    if eff >= 6:
        for i in range(0, ch + 1, eff):
            cv2.line(img, (0, i), (cw, i), CLR_GRID, 1)
        for j in range(0, cw + 1, eff):
            cv2.line(img, (j, 0), (j, ch), CLR_GRID, 1)

    return img, eff


def _px(row, col, eff):
    """栅格坐标 → 渲染像素中心"""
    return (col * eff + eff // 2, row * eff + eff // 2)


def draw_path(grid, path, start, goal):
    """
    单条路径预览。返回 BGR 图，可直接 cv2.imshow。
    - 障碍=黑，空闲=白，栅格线=浅灰
    - 路径=青色折线，起点=绿圆，终点=红十字
    """
    vis, eff = _render(grid)

    if path:
        pts = [_px(r, c, eff) for r, c in path]
        for i in range(len(pts) - 1):
            cv2.line(vis, pts[i], pts[i + 1], CLR_PATH, max(2, eff // 6))

    # 起点（绿圆）
    sp = _px(start[0], start[1], eff)
    cv2.circle(vis, sp, max(6, eff), CLR_START, -1)
    cv2.circle(vis, sp, max(6, eff), (0, 100, 0), 2)
    cv2.putText(vis, "S", (sp[0] + 8, sp[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 0), 2)

    # 终点（红十字）
    gp = _px(goal[0], goal[1], eff)
    cv2.drawMarker(vis, gp, CLR_GOAL, markerType=cv2.MARKER_CROSS,
                   markerSize=max(10, eff + 4), thickness=2)
    cv2.putText(vis, "G", (gp[0] + 8, gp[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 200), 2)

    # 信息条
    h = vis.shape[0]
    cv2.putText(vis, f"Path: {len(path) - 1} segments | Press any key to close",
                (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return vis


def draw_comparison(grid, results, start, goal):
    """
    多算法对比预览。返回 BGR 图，可直接 cv2.imshow。
    不同算法用不同颜色叠加。
    """
    vis, eff = _render(grid)

    colors = [CLR_PATH, (255, 0, 255), (0, 255, 255), (0, 165, 255), (50, 255, 50)]
    legend_y = 30

    for i, (name, data) in enumerate(results.items()):
        p = data.get('path', [])
        if not p:
            continue
        color = colors[i % len(colors)]
        pts = [_px(r, c, eff) for r, c in p]
        for j in range(len(pts) - 1):
            cv2.line(vis, pts[j], pts[j + 1], color, max(2, eff // 6))

        steps = len(p) - 1
        t = data.get('time', 0)
        label = f"{name}: {steps} steps, {t:.3f}s"
        cv2.putText(vis, label, (10, legend_y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, color, 1, cv2.LINE_AA)
        legend_y += 20

    sp = _px(start[0], start[1], eff)
    gp = _px(goal[0], goal[1], eff)
    cv2.circle(vis, sp, max(6, eff), CLR_START, -1)
    cv2.drawMarker(vis, gp, CLR_GOAL, markerType=cv2.MARKER_CROSS,
                   markerSize=max(10, eff + 4), thickness=2)
    cv2.putText(vis, "S", (sp[0] + 8, sp[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 0), 2)
    cv2.putText(vis, "G", (gp[0] + 8, gp[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 200), 2)

    h = vis.shape[0]
    cv2.putText(vis, "Algorithm Comparison | Press any key to close",
                (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return vis


def show(title, img):
    """统一的 cv2 弹窗封装，WINDOW_NORMAL 兼容性最好。"""
    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    cv2.imshow(title, img)
    print(f"[Preview] {title} — press any key in the window to continue...")
    cv2.waitKey(0)
    cv2.destroyWindow(title)
