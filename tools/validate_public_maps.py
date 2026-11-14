"""Reproducible random official tasks across every imported map, with bounded workers."""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import json
import random
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from import_movingai import parse_map


def evaluate(case):
    from path_planner.src.planner_factory import create_planner
    from path_planner.src.grid_utils import line_of_sight
    from path_postprocess import process_path
    _, _, rows = parse_map((ROOT / case['map_file']).read_bytes())
    grid = (np.array([list(row) for row in rows]) == 0).astype(np.uint8)
    start, goal = tuple(case['start']), tuple(case['goal'])
    begin = time.perf_counter()
    planner = create_planner(case['algorithm'], grid, case['parameters'], seed=case['seed'])
    path = planner.plan(start, goal)
    elapsed = time.perf_counter() - begin
    valid = bool(path) and path[0] == start and path[-1] == goal and all(
        line_of_sight(grid, *grid.shape, a, b) for a, b in zip(path, path[1:]))
    result = dict(outcome='success' if valid else 'NO_PATH' if not path else 'invalid_path',
                  planning_s=elapsed, nodes=len(path), search_diagnostics=getattr(planner, 'diagnostics', {}))
    if valid:
        processed = process_path(grid, path, start=start, goal=goal, grid_size_m=1.0)
        result.update(postprocess=processed['status'], errors=processed['errors'])
        if processed['status'] != 'success':
            result['outcome'] = 'postprocess_failed'
    print(json.dumps(result))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--child')
    parser.add_argument('--output', default='docs/verification/all-maps-three-tasks.json')
    parser.add_argument('--timeout', type=int, default=60)
    parser.add_argument('--planner-config', help='Use algorithm_params from this YAML instead of baseline parameters')
    args = parser.parse_args()
    if args.child:
        evaluate(json.loads(args.child))
        return
    manifest = json.loads((ROOT / 'datasets/movingai/manifest.json').read_text(encoding='utf-8'))
    cases = []
    config = yaml.safe_load(Path(args.planner_config).read_text(encoding='utf-8')) if args.planner_config else None
    rng = random.Random(20261114)
    for item in manifest['maps']:
        tasks = [line.split() for line in (ROOT / item['scenarios']).read_text().splitlines()[1:] if line.strip()]
        for group, index in enumerate(rng.sample(range(len(tasks)), 3), 1):
            task = tasks[index]
            for algorithm in ['astar', 'dijkstra', 'prm', 'rrt', 'rrt_star']:
                params = dict(num_samples=20000, k_neighbors=150, max_edge_len=30) if algorithm == 'prm' else (
                    dict(step_size=1, max_iter=200000, goal_sample_rate=0.1) if algorithm in ('rrt', 'rrt_star') else {})
                if algorithm == 'rrt_star':
                    params['search_radius'] = 10
                if config is not None:
                    from path_planner.src.planner_factory import resolve_algorithm_parameters
                    params = resolve_algorithm_parameters(algorithm, config['path_planner'].get('algorithm_params', {}).get(algorithm, {}))
                cases.append(dict(map=item['name'],map_file=item['map'],group=group,scenario_index=index,
                                  reference_length_octile=float(task[8]),start=[int(task[5]),int(task[4])],
                                  goal=[int(task[7]),int(task[6])],algorithm=algorithm,parameters=params,seed=42+group))
    target = ROOT / args.output
    report = dict(selection_seed=20261114, selection='three uniform official scenario indices per map, no distance filter',
                  timeout_s=args.timeout, planner_config=config, expected=len(cases), results=[])
    def run(case):
        try:
            proc = subprocess.run([sys.executable,__file__,'--child',json.dumps(case)],cwd=ROOT,capture_output=True,timeout=args.timeout)
            result = json.loads(proc.stdout) if proc.returncode == 0 else dict(outcome='error',stderr=proc.stderr.decode(errors='replace')[-1500:])
        except subprocess.TimeoutExpired:
            result = dict(outcome='timeout')
        return dict(**case,**result)
    with ThreadPoolExecutor(max_workers=3) as pool:
        for future in as_completed([pool.submit(run,case) for case in cases]):
            row = future.result()
            report['results'].append(row)
            report['complete'] = len(report['results']) == len(cases)
            report['all_passed'] = report['complete'] and all(r['outcome']=='success' for r in report['results'])
            target.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
            print(len(report['results']),row['map'],row['group'],row['algorithm'],row['outcome'],flush=True)


if __name__ == '__main__':
    main()
