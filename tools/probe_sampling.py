"""Replay an official ~80-cell task directly through a planner (no run logs)."""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from import_movingai import parse_map
from path_planner.src.planner_factory import create_planner
from path_planner.src.grid_utils import line_of_sight


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', default='random512-10-0')
    parser.add_argument('--algorithm', default='prm')
    parser.add_argument('--params', default='{}')
    parser.add_argument('--require-path', action='store_true')
    parser.add_argument('--output', help='Save compact results and raw path as JSON')
    parser.add_argument('--postprocess', action='store_true', help='Check default and unsmoothed postprocessing')
    args = parser.parse_args()
    manifest = json.loads((ROOT / 'datasets/movingai/manifest.json').read_text(encoding='utf-8'))
    item = next(item for item in manifest['maps'] if item['name'] == args.map)
    _, _, rows = parse_map((ROOT / item['map']).read_bytes())
    grid = (np.asarray([list(row) for row in rows]) == 0).astype(np.uint8)
    tasks = [line.split() for line in (ROOT / item['scenarios']).read_text().splitlines()[1:] if line.strip()]
    index, task = min(enumerate(tasks), key=lambda pair: abs(float(pair[1][8]) - 80))
    start, goal = (int(task[5]), int(task[4])), (int(task[7]), int(task[6]))
    planner = create_planner(args.algorithm, grid, params=json.loads(args.params), seed=42)
    extra = {}
    if args.algorithm in ('rrt', 'rrt_star'):
        original_nearest = planner._nearest_node
        def capture_tree(nodes, point):
            extra.update(tree_nodes=len(nodes), unique_nodes=len({node[0] for node in nodes}),
                         nearest_goal=round(min(planner._distance(node[0], goal) for node in nodes), 3))
            return original_nearest(nodes, point)
        planner._nearest_node = capture_tree
    if args.algorithm == 'prm':
        original = planner._build_roadmap
        def capture(points):
            roadmap, points = original(points)
            pending = [points.index(start)]
            reached = set(pending)
            while pending:
                for neighbor in roadmap[pending.pop()]:
                    if neighbor not in reached:
                        reached.add(neighbor)
                        pending.append(neighbor)
            extra.update(samples=len(points)-2, start_degree=len(set(roadmap[points.index(start)])),
                         goal_degree=len(set(roadmap[points.index(goal)])), start_component=len(reached))
            return roadmap, points
        planner._build_roadmap = capture
    begin = time.perf_counter()
    path = planner.plan(start, goal)
    valid = bool(path) and path[0] == start and path[-1] == goal and all(
        line_of_sight(grid, *grid.shape, a, b) for a, b in zip(path, path[1:]))
    result = dict(map=args.map, scenario=index, start=start, goal=goal,
                         algorithm=args.algorithm, params=json.loads(args.params), seed=42,
                         found=bool(path), valid=valid, nodes=len(path),
                         elapsed_s=round(time.perf_counter()-begin, 4), **extra)
    if args.postprocess and valid:
        from path_postprocess import process_path
        result['postprocess'] = {}
        for label, parameters in [('default', {}), ('unsmoothed', {'smooth': False})]:
            processed = process_path(grid, path, grid_size_m=1.0, start=start, goal=goal,
                                     parameters=parameters)
            result['postprocess'][label] = dict(status=processed['status'], errors=processed['errors'])
    print(json.dumps(result))
    if args.output:
        target = Path(args.output)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(json.dumps(dict(**result, path=path), ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    return 1 if args.require_path and not valid else 0


if __name__ == '__main__':
    raise SystemExit(main())
