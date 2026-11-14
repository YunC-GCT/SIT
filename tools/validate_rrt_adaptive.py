"""Run the explicit adaptive RRT variant on the same 36 baseline tasks."""
from concurrent.futures import ThreadPoolExecutor, as_completed
import json
import subprocess
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]


def main():
    baseline = json.loads((ROOT / 'docs/verification/all-maps-latest-config.json').read_text(encoding='utf-8'))
    parameters = yaml.safe_load((ROOT / 'path_planner/config.yaml').read_text(encoding='utf-8'))['path_planner']['algorithm_params']['rrt_adaptive']
    cases = [row for row in baseline['results'] if row['algorithm'] == 'rrt']
    report = dict(algorithm='rrt_adaptive', parameters=parameters, timeout_s=30,
                  baseline='all-maps-latest-config.json', expected=36, results=[])
    target = ROOT / 'docs/verification/rrt-adaptive-full-test.json'
    def run(row):
        case = {key:row[key] for key in ['map', 'map_file', 'group', 'scenario_index',
                'reference_length_octile', 'start', 'goal', 'seed']}
        case.update(algorithm='rrt_adaptive', parameters=parameters)
        try:
            proc = subprocess.run([sys.executable, str(ROOT/'tools/validate_public_maps.py'),
                                   '--child', json.dumps(case)], cwd=ROOT, capture_output=True, timeout=30)
            result = json.loads(proc.stdout) if proc.returncode == 0 else dict(
                outcome='error', stderr=proc.stderr.decode(errors='replace')[-1500:])
        except subprocess.TimeoutExpired:
            result = dict(outcome='timeout')
        return dict(**case, baseline_outcome=row['outcome'], **result)
    with ThreadPoolExecutor(max_workers=3) as pool:
        for future in as_completed([pool.submit(run, row) for row in cases]):
            row = future.result()
            report['results'].append(row)
            report['complete'] = len(report['results']) == 36
            report['all_passed'] = report['complete'] and all(x['outcome']=='success' for x in report['results'])
            target.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
            print(len(report['results']),row['map'],row['group'],row['outcome'],flush=True)


if __name__ == '__main__':
    main()
