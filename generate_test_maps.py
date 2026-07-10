"""Generate 5 test maps for path planning, saved to map_pipeline/data/input/."""
import numpy as np
import cv2
import os

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "map_pipeline", "data", "input")
os.makedirs(OUT_DIR, exist_ok=True)


def save(name, grid_2d):
    img = ((1 - grid_2d) * 255).astype(np.uint8)
    path = os.path.join(OUT_DIR, f"{name}.png")
    ret, buf = cv2.imencode('.png', img)
    if not ret:
        raise RuntimeError(f"encode failed: {name}")
    with open(path, 'wb') as f:
        f.write(buf.tobytes())
    print(f"  [OK] {name}.png")


def make_grid(n, m=0):
    return np.zeros((n, m or n), dtype=np.uint8)


print("Generating test maps...")

# --- Map 1: scattered blocks ---
g = make_grid(80)
rng = np.random.RandomState(42)
for _ in range(120):
    r, c = rng.randint(5, 75), rng.randint(5, 75)
    s = rng.randint(1, 5)
    g[r:r+s, c:c+s] = 1
g[0:10, 0:10] = 0; g[70:80, 70:80] = 0
save("01_scattered", g)

# --- Map 2: maze-like corridors ---
g = make_grid(80)
g[0:2, :] = 1; g[78:80, :] = 1
g[:, 0:2] = 1; g[:, 78:80] = 1
for y in [15, 30, 45, 60]:
    g[y:y+2, 5:30] = 1; g[y:y+2, 50:75] = 1
for x in [25, 55]:
    g[8:18, x:x+2] = 1; g[32:42, x:x+2] = 1
    g[48:58, x:x+2] = 1; g[62:72, x:x+2] = 1
g[3:8, 3:8] = 0; g[72:78, 72:78] = 0
save("02_maze", g)

# --- Map 3: narrow winding path ---
g = np.ones((80, 80), dtype=np.uint8)
for y in range(76):
    if y < 25:       g[y, 10:15] = 0
    elif y < 40:     g[y, 30:35] = 0
    elif y < 55:     g[y, 50:55] = 0
    else:            g[y, 65:70] = 0
g[10:15, 15:30] = 0; g[30:35, 35:50] = 0
g[50:55, 55:65] = 0
g[3:8, 8:13] = 0; g[72:78, 63:68] = 0
save("03_narrow", g)

# --- Map 4: four rooms ---
g = make_grid(80)
g[0:2, :] = 1; g[78:80, :] = 1
g[:, 0:2] = 1; g[:, 78:80] = 1
g[38:42, :] = 1; g[:, 38:42] = 1
# doorways
g[38:42, 10:14] = 0; g[38:42, 30:34] = 0
g[38:42, 48:52] = 0; g[38:42, 66:70] = 0
g[10:14, 38:42] = 0; g[30:34, 38:42] = 0
g[48:52, 38:42] = 0; g[66:70, 38:42] = 0
g[5:10, 5:10] = 0; g[70:76, 70:76] = 0
save("04_rooms", g)

# --- Map 5: spiral ---
g = make_grid(80)
cx, cy = 40, 40
for i in range(1, 8):
    r = i * 5
    g[cy-r:cy-r+1, cx-r:cx+r+1] = 1
    if i % 4 == 1: g[cy-r, cx-r:cx-r+5] = 0
    g[cy-r:cy+r+2, cx+r:cx+r+1] = 1
    if i % 4 == 2: g[cy-r:cy-r+5, cx+r] = 0
    g[cy+r:cy+r+1, cx-r:cx+r+1] = 1
    if i % 4 == 3: g[cy+r, cx+r-5:cx+r] = 0
    g[cy-r:cy+r+2, cx-r:cx-r+1] = 1
    if i % 4 == 0: g[cy+r-5:cy+r, cx-r] = 0
g[0:6, 0:6] = 0; g[74:80, 74:80] = 0
g[0:cy-34, 3:6] = 0; g[cy+35:79, 75:78] = 0
save("05_spiral", g)

print(f"\nDone. {OUT_DIR}/")
