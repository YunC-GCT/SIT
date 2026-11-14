"""对已选任务进行限时参数探测，并保留失败结果。"""
from concurrent.futures import ThreadPoolExecutor, as_completed
import argparse
from typing import Any
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main():
    """构造分场景采样参数候选，限时运行并保存调参比较记录。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--confirm', action='store_true', help='Validate selected PRM profile on all original tasks')
    args = parser.parse_args()
    baseline = json.loads((ROOT / 'docs/verification/all-maps-three-tasks.json').read_text(encoding='utf-8'))
    probes = []
    for name in ['8room_000', 'maze512-4-0', 'maze512-16-0', 'random512-20-0']:
        for algorithm in ['prm', 'rrt', 'rrt_star']:
            base = next(x for x in baseline['results'] if x['map'] == name and x['algorithm'] == algorithm and x['group'] == 1)
            if algorithm == 'prm':
                variants = [dict(num_samples=20000,k_neighbors=15,max_edge_len=15),
                            dict(num_samples=6000,k_neighbors=15,max_edge_len=40)]
            else:
                variants = [dict(max_iter=20000,step_size=step,goal_sample_rate=0.05) for step in (2,5)]
                if algorithm == 'rrt_star':
                    for params in variants:
                        params['search_radius'] = 6
            for params in variants:
                probes.append({**{k:base[k] for k in ['map','map_file','group','scenario_index','reference_length_octile','start','goal','algorithm','seed']},
                               'parameters':params,'baseline_outcome':base['outcome']})
    if args.confirm:
        probes = []
        for base in baseline['results']:
            if base['algorithm'] != 'prm':
                continue
            narrow = base['map'] in ('8room_000', '16room_000', 'maze512-4-0', 'maze512-8-0', 'random512-40-0')
            params = dict(num_samples=20000 if narrow else 6000, k_neighbors=15, max_edge_len=15 if narrow else 40)
            probes.append({**{k:base[k] for k in ['map','map_file','group','scenario_index','reference_length_octile','start','goal','algorithm','seed']},
                           'parameters':params,'baseline_outcome':base['outcome']})
    report: dict[str, Any] = dict(scope='selected PRM profiles on all 36 original tasks' if args.confirm else 'parameter screening only; same official tasks/seeds as baseline; not full acceptance',timeout_s=15,results=[])
    target = ROOT / ('docs/verification/sampling-prm-profile-validation.json' if args.confirm else 'docs/verification/sampling-parameter-screening.json')
    def run(case):
        """执行单个参数候选的子进程并收集状态、耗时或超时信息。"""
        try:
            proc = subprocess.run([sys.executable,str(ROOT/'tools/validate_public_maps.py'),'--child',json.dumps(case)],
                                  cwd=ROOT,capture_output=True,timeout=15)
            result = json.loads(proc.stdout) if proc.returncode == 0 else dict(outcome='error',stderr=proc.stderr.decode(errors='replace')[-1000:])
        except subprocess.TimeoutExpired:
            result = dict(outcome='timeout')
        return dict(**case,**result)
    with ThreadPoolExecutor(max_workers=3) as pool:
        for future in as_completed([pool.submit(run,case) for case in probes]):
            row = future.result()
            report['results'].append(row)
            report['complete'] = len(report['results']) == len(probes)
            target.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
            print(len(report['results']),row['map'],row['algorithm'],row['parameters'],row['outcome'],flush=True)


if __name__ == '__main__':
    main()
