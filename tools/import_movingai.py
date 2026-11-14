"""Download a representative Moving AI subset and preserve its exact grid.

Run from the project root: python tools/import_movingai.py
Uses only the Python standard library; never extracts arbitrary ZIP paths.
"""

from __future__ import annotations

import hashlib
import io
import json
import struct
import urllib.request
import zipfile
import zlib
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SELECTION = {
    "room": [f"{size}room_000" for size in (8, 16, 32, 64)],
    "maze": [f"maze512-{width}-0" for width in (4, 8, 16, 32)],
    "random": [f"random512-{density}-0" for density in (10, 20, 30, 40)],
}


def download_member(url: str, suffix: str) -> bytes:
    with urllib.request.urlopen(url, timeout=60) as response:
        archive_data = response.read()
    with zipfile.ZipFile(io.BytesIO(archive_data)) as archive:
        members = [name for name in archive.namelist() if name.endswith(suffix)]
        if len(members) != 1:
            raise ValueError(f"expected one {suffix} member: {url}")
        return archive.read(members[0])


def parse_map(data: bytes) -> tuple[int, int, list[bytes]]:
    lines = data.decode("ascii").splitlines()
    if lines[0] != "type octile" or lines[3] != "map":
        raise ValueError("unsupported Moving AI header")
    height = int(lines[1].split()[1])
    width = int(lines[2].split()[1])
    rows = lines[4:]
    if len(rows) != height or any(len(row) != width for row in rows):
        raise ValueError("map dimensions do not match header")
    symbols = set("".join(rows))
    if not symbols <= set(".G@OT"):
        raise ValueError(f"unsupported terrain: {symbols}")
    pixels = [bytes(255 if cell in ".G" else 0 for cell in row) for row in rows]
    return width, height, pixels


def png_bytes(width: int, height: int, rows: list[bytes]) -> bytes:
    def chunk(kind: bytes, data: bytes) -> bytes:
        return (struct.pack(">I", len(data)) + kind + data
                + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF))

    header = struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0)
    raw = b"".join(b"\0" + row for row in rows)
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", header)
            + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b""))


def import_one(item: tuple[str, str]) -> dict:
    family, name = item
    base = f"https://movingai.com/benchmarks/{family}/"
    map_url = f"{base}{name}.map.zip"
    scenario_url = f"{base}{name}.map-scen.zip"
    original = download_member(map_url, ".map")
    scenarios = download_member(scenario_url, ".scen")
    width, height, rows = parse_map(original)
    tasks = []
    for line in scenarios.decode("ascii").splitlines()[1:]:
        if not line.strip():
            continue
        fields = line.split()
        if len(fields) != 9 or fields[1] != f"{name}.map":
            raise ValueError(f"unexpected scenario record for {name}")
        if (int(fields[2]), int(fields[3])) != (width, height):
            raise ValueError("scenario would rescale the source map")
        start = [int(fields[5]), int(fields[4])]
        goal = [int(fields[7]), int(fields[6])]
        for row, col in (start, goal):
            if not (0 <= row < height and 0 <= col < width and rows[row][col] == 255):
                raise ValueError("scenario endpoint is not a free cell")
        tasks.append({"index": len(tasks), "start": start, "goal": goal,
                      "reference_length_octile": float(fields[8])})
    if not tasks:
        raise ValueError("scenario contains no tasks")
    source_dir = ROOT / "datasets" / "movingai" / family
    source_dir.mkdir(parents=True, exist_ok=True)
    (source_dir / f"{name}.map").write_bytes(original)
    (source_dir / f"{name}.scen").write_bytes(scenarios)
    image_path = ROOT / "map_pipeline" / "data" / "input" / f"{family}-{name}.png"
    image_path.parent.mkdir(parents=True, exist_ok=True)
    image = png_bytes(width, height, rows)
    image_path.write_bytes(image)
    print(f"Imported {family}/{name}: {width}x{height}, {len(tasks)} tasks", flush=True)
    return {
        "family": family, "name": name, "width": width, "height": height,
        "image": image_path.relative_to(ROOT).as_posix(),
        "map": (source_dir / f"{name}.map").relative_to(ROOT).as_posix(),
        "scenarios": (source_dir / f"{name}.scen").relative_to(ROOT).as_posix(),
        "map_url": map_url, "scenario_url": scenario_url,
        "sha256": {"map": hashlib.sha256(original).hexdigest(),
                   "scenarios": hashlib.sha256(scenarios).hexdigest(),
                   "image": hashlib.sha256(image).hexdigest()},
        "task_count": len(tasks), "example_task": tasks[0],
    }


def main() -> None:
    items = [(family, name) for family, names in SELECTION.items() for name in names]
    with ThreadPoolExecutor(max_workers=4) as pool:
        maps = list(pool.map(import_one, items))
    manifest = {
        "dataset": "Moving AI 2D Grid Benchmarks",
        "downloaded_at": datetime.now(timezone.utc).isoformat(),
        "source": "https://movingai.com/benchmarks/grids.html",
        "license": "https://opendatacommons.org/licenses/by/1-0/",
        "citation": "Nathan Sturtevant, Benchmarks for Grid-Based Pathfinding, IEEE TCIAIG 4(2), 144-148, 2012.",
        "coordinate_system": "row,col; scenario x,y is converted to y,x",
        "scale": "No real-world metric calibration; grid_size is an experiment convention.",
        "maps": maps,
    }
    target = ROOT / "datasets" / "movingai" / "manifest.json"
    target.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
