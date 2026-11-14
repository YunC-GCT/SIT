"""Bounded before/after comparison using a local pre-fix snapshot."""
import argparse
import importlib
from typing import Any
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / 'tools'), os.environ.get('SAMPLING_BASELINE_PARENT', str(ROOT / '.venv'))]
from import_movingai import parse_map


def child(case, version):
    factory = importlib.import_module('planner_before.planner_factory' if version == 'before' else 'path_planner.src.planner_factory')
    manifest = json.loads((ROOT / 'datasets/movingai/manifest.json').read_text(encoding='utf-8'))
    item = next(x for x in manifest['maps'] if x['name'] == case['map'])
    _, _, rows = parse_map((ROOT / item['map']).read_bytes())
    grid = (np.array([list(row) for row in rows]) == 0).astype(np.uint8)
    tasks = [line.split() for line in (ROOT / item['scenarios']).read_text().splitlines()[1:] if line.strip()]
    task = min(tasks, key=lambda t: abs(float(t[8])-80))
    start, goal = (int(task[5]), int(task[4])), (int(task[7]), int(task[6]))
    planner = factory.create_planner(case['algorithm'], grid, case['params'], seed=42)
    begin = time.perf_counter()
    path = planner.plan(start, goal)
    elapsed = time.perf_counter()-begin
    from path_planner.src.grid_utils import line_of_sight
    valid = bool(path) and path[0] == start and path[-1] == goal and all(line_of_sight(grid, int(grid.shape[0]), int(grid.shape[1]), a, b) for a, b in zip(path, path[1:]))
    from path_postprocess import process_path
    post = process_path(grid, path, start=start, goal=goal, grid_size_m=1.0) if valid else None
    print(json.dumps(dict(found=bool(path), valid=valid, elapsed_s=elapsed, nodes=len(path),
                         length_grid=sum(float(np.linalg.norm(np.subtract(a,b))) for a,b in zip(path,path[1:])),
                         postprocess=post['status'] if post else None, path=path)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--child')
    parser.add_argument('--version')
    parser.add_argument('--output', default='docs/verification/sampling-fixes.json')
    args = parser.parse_args()
    if args.child:
        child(json.loads(args.child), args.version)
        return
    cases = []
    for name in ['8room_000', 'maze512-4-0', 'random512-10-0']:
        for algorithm in ['prm', 'rrt', 'rrt_star']:
            cases.append(dict(map=name, algorithm=algorithm, params={}, label='default'))
    for name, algorithm, params in [
        ('8room_000','prm',dict(num_samples=20000,k_neighbors=150)),
        ('maze512-4-0','prm',dict(num_samples=20000,k_neighbors=150)),
        ('maze512-4-0','rrt_star',dict(step_size=1,max_iter=200000)),
        ('random512-10-0','prm',dict(num_samples=3000)),
    ]:
        cases.append(dict(map=name,algorithm=algorithm,params=params,label='tuned'))
    results = []
    target = ROOT / args.output
    for case in cases:
        row: dict[str, Any] = dict(**case, seed=42)
        for version in ['before', 'after']:
            try:
                proc = subprocess.run([sys.executable, __file__, '--child', json.dumps(case), '--version', version],
                                      capture_output=True, timeout=30, cwd=ROOT)
                row[version] = json.loads(proc.stdout) if proc.returncode == 0 else dict(error=proc.stderr.decode(errors='replace')[-1000:])
            except subprocess.TimeoutExpired:
                row[version] = dict(timeout_s=30)
            print(case['map'],case['algorithm'],case['label'],version,
                  {k:v for k,v in row[version].items() if k!='path'},flush=True)
        results.append(row)
        target.write_text(json.dumps(dict(timeout_s=30,results=results),ensure_ascii=False,indent=2)+'\n',encoding='utf-8')


if __name__ == '__main__':
    main()
