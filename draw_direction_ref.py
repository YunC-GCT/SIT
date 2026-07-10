"""Generate a grid diagram explaining robot direction mapping. Not part of the pipeline."""
import cv2, numpy as np, os

OUT = os.path.join(os.path.dirname(__file__), "docs")
os.makedirs(OUT, exist_ok=True)

W, H = 700, 700
img = np.full((H, W, 3), 255, dtype=np.uint8)  # white bg


def arrow(im, x0, y0, dx, dy, color, thickness=2, tip=12):
    x1, y1 = x0 + dx, y0 + dy
    cv2.arrowedLine(im, (x0, y0), (x1, y1), color, thickness, tipLength=tip/(np.hypot(dx,dy)+1e-5))


def text(im, x, y, s, color, scale=0.6, thick=2, center=True):
    (w, h), _ = cv2.getTextSize(s, cv2.FONT_HERSHEY_SIMPLEX, scale, thick)
    if center: x -= w // 2
    cv2.putText(im, s, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick)


# ── Grid ──
cx, cy = 350, 350
cell = 60
for i in range(-4, 5):
    x = cx + i * cell
    cv2.line(img, (x, cy - 4*cell), (x, cy + 4*cell), (220, 220, 220), 1)
    y = cy + i * cell
    cv2.line(img, (cx - 4*cell, y), (cx + 4*cell, y), (220, 220, 220), 1)

# ── Robot at center (facing EAST) ──
r = 22
cv2.circle(img, (cx, cy), r, (0, 120, 255), -1)
# direction indicator (triangle pointing EAST)
pts = np.array([[cx+r, cy], [cx+r//2, cy-12], [cx+r//2, cy+12]], dtype=np.int32)
cv2.fillPoly(img, [pts], (0, 200, 0))
text(img, cx, cy-35, "ROBOT", (0, 100, 200), 0.5, 2)

# ── Four cardinal arrows + labels ──
# EAST = forward
arrow(img, cx, cy, 2*cell, 0, (0, 180, 0), 3)
text(img, cx+3*cell, cy-15, "EAST = forward", (0, 140, 0), 0.5, 2)
text(img, cx+3*cell, cy+5, "(+vx, 0)", (0, 140, 0), 0.45, 1)

# SOUTH = strafe right
arrow(img, cx, cy, 0, 2*cell, (0, 0, 200), 3)
text(img, cx, cy+3*cell+15, "SOUTH = strafe right", (0, 0, 180), 0.5, 2)
text(img, cx, cy+3*cell+35, "(0, -vy)", (0, 0, 180), 0.45, 1)

# WEST = backward
arrow(img, cx, cy, -2*cell, 0, (180, 0, 0), 3)
text(img, cx-3*cell, cy-15, "WEST = backward", (160, 0, 0), 0.5, 2)
text(img, cx-3*cell, cy+5, "(-vx, 0)", (160, 0, 0), 0.45, 1)

# NORTH = strafe left
arrow(img, cx, cy, 0, -2*cell, (120, 0, 120), 3)
text(img, cx, cy-3*cell-5, "NORTH = strafe left", (100, 0, 100), 0.5, 2)
text(img, cx, cy-3*cell+15, "(0, +vy)", (100, 0, 100), 0.45, 1)

# ── Grid axis labels ──
text(img, cx, H-10, "col →  (dc > 0)", (100, 100, 100), 0.4, 1)
text(img, 10, cy, "row ↓ (dr > 0)", (100, 100, 100), 0.4, 1, center=False)

# ── Legend box ──
lx, ly = 20, 20
cv2.rectangle(img, (lx, ly), (lx+280, ly+110), (200, 200, 200), 1)
text(img, lx+140, ly+20, "Mecanum Omnidirectional", (0, 0, 0), 0.5, 2)
text(img, lx+10, ly+45, "Robot faces EAST (initial_heading)", (80, 80, 80), 0.4, 1, center=False)
text(img, lx+10, ly+65, "angular.z = 0  |  no rotation", (80, 80, 80), 0.4, 1, center=False)
text(img, lx+10, ly+85, "All directions = (vx, vy) directly", (80, 80, 80), 0.4, 1, center=False)
text(img, lx+10, ly+105, "Grid coordinates: row 0 = top", (80, 80, 80), 0.4, 1, center=False)

path = os.path.join(OUT, "robot_direction_reference.png")
ret, buf = cv2.imencode('.png', img)
with open(path, 'wb') as f:
    f.write(buf.tobytes())
print(f"Saved: {path}")
