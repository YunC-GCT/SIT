"""在无硬件环境检查公开地图与真实模块 CLI，保留精简 JSON 报告。

详细运行数据写入临时目录，验证结束后自动删除。
"""
from __future__ import annotations

import argparse
import hashlib
import json
import platform
import subprocess
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from import_movingai import parse_map
from map_pipeline.process import prepare_grid_map


def invoke(arguments: list[str], result: Path, timeout: int = 45) -> dict:
    """限时执行真实模块 CLI，读取结果并区分超时、缺失结果和模块返回状态。"""
    try:
        proc = subprocess.run(
            [sys.executable, *arguments], cwd=ROOT, capture_output=True,
            timeout=timeout,
        )
    except subprocess.TimeoutExpired:
        return {"outcome": "timeout", "timeout_s": timeout}
    if not result.exists():
        return {"outcome": "missing_result", "exit_code": proc.returncode,
                "stderr": proc.stderr.decode(errors="replace")[-1500:]}
    payload = json.loads(result.read_text(encoding="utf-8"))
    return {"outcome": payload["status"], "exit_code": proc.returncode,
            "errors": payload.get("errors", []), "payload": payload}


def compact(result: dict) -> dict:
    """去除完整 payload，仅保留状态与错误等精简验收信息。"""
    return {key: value for key, value in result.items() if key != "payload"}


def main() -> int:
    """核对公开地图与真实模块链路，把精简离线验收报告保存到指定文件。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    manifest = json.loads((ROOT / "datasets/movingai/manifest.json").read_text(encoding="utf-8"))
    config = yaml.safe_load((ROOT / "map_pipeline/config.yaml").read_text(encoding="utf-8"))
    checks = []
    sampling = []
    with tempfile.TemporaryDirectory(prefix="sit-offline-") as temporary:
        run_root = Path(temporary)
        for item in manifest["maps"]:
            width, height, rows = parse_map((ROOT / item["map"]).read_bytes())
            expected = (np.asarray([list(row) for row in rows]) == 0).astype(np.uint8)
            image = cv2.imdecode(np.fromfile(ROOT / item["image"], dtype=np.uint8), cv2.IMREAD_COLOR)
            actual = prepare_grid_map(image, config).grid
            hashes = all(
                hashlib.sha256((ROOT / item[path_key]).read_bytes()).hexdigest() == item["sha256"][hash_key]
                for path_key, hash_key in (("map", "map"), ("scenarios", "scenarios"), ("image", "image"))
            )
            # Select a nontrivial official task near 80 cells, preserving x/y -> row/col.
            tasks = [line.split() for line in (ROOT / item["scenarios"]).read_text().splitlines()[1:] if line.strip()]
            index, task = min(enumerate(tasks), key=lambda pair: abs(float(pair[1][8]) - 80))
            start, goal = f"{task[5]},{task[4]}", f"{task[7]},{task[6]}"
            run_id = item["name"]
            base = ["main.py", item["image"], "--headless", "--no-drive",
                    "--start", start, "--goal", goal, "--seed", "42",
                    "--run-root", temporary, "--run-id", run_id]
            outcome = invoke(base + ["--mode", "research", "--algorithms", "astar,dijkstra"],
                             run_root / run_id / "path_postprocess/result.json")
            pipeline_manifest = run_root / run_id / "manifest.json"
            batch = json.loads(pipeline_manifest.read_text(encoding="utf-8")) if pipeline_manifest.exists() else {}
            check = {"map": item["name"], "family": item["family"],
                     "shape": [height, width], "exact_grid": bool(np.array_equal(expected, actual)),
                     "hashes_match": hashes, "scenario_index": index, "start": start, "goal": goal,
                     "pipeline": compact(outcome), "batch_status": batch.get("status"),
                     "motion_status": batch.get("stages", {}).get("motion_control", {}).get("status")}
            checks.append(check)
            print(f"{item['name']}: exact={check['exact_grid']} pipeline={outcome['outcome']}", flush=True)
            if item["name"] in {"8room_000", "maze512-4-0", "random512-10-0"}:
                for algorithm in ("prm", "rrt", "rrt_star"):
                    sample_id = run_id + "-" + algorithm
                    sample_args = base.copy()
                    sample_args[-1] = sample_id
                    sample = invoke(sample_args + ["--algorithm", algorithm],
                                    run_root / sample_id / "path_planner/result.json")
                    post_path = run_root / sample_id / "path_postprocess/result.json"
                    planner_metrics = sample.get("payload", {}).get("outputs", {}).get("metrics", {}).get("values", {})
                    sampling.append({"map": item["name"], "algorithm": algorithm, "seed": 42,
                                     "planner": compact(sample),
                                     "parameters": sample.get("payload", {}).get("parameters", {}),
                                     "metrics": planner_metrics,
                                     "postprocess_status": json.loads(post_path.read_text(encoding="utf-8"))["status"] if post_path.exists() else None})
                    print(f"  {algorithm}: {sample['outcome']}", flush=True)
        acceptance = []
        for mode in ("normal", "research"):
            run_id = "acceptance-" + mode
            command = ["tools/acceptance.py", "--mode", mode, "--run-root", temporary, "--run-id", run_id]
            if mode == "research":
                command += ["--algorithms", "astar,dijkstra"]
            result = invoke(command, run_root / run_id / "acceptance/result.json")
            payload = result.get("payload", {})
            acceptance.append({"mode": mode, **compact(result),
                               "checks": payload.get("outputs", {}).get("checks", []),
                               "manual_hardware_status": payload.get("outputs", {}).get("manual_hardware_acceptance", {}).get("status")})
        failures = []
        for name, extra in (("missing", ["does-not-exist.png", "--start", "0,0", "--goal", "1,1"]),
                            ("missing-points", [manifest["maps"][0]["image"]])):
            run_id = "negative-" + name
            result = subprocess.run([sys.executable, "main.py", *extra, "--headless", "--no-drive", "--run-root", temporary, "--run-id", run_id],
                                    cwd=ROOT, capture_output=True, timeout=30)
            batch = json.loads((run_root / run_id / "manifest.json").read_text(encoding="utf-8"))
            failures.append({"case": name, "exit_code": result.returncode,
                             "batch_status": batch["status"], "stages": list(batch["stages"]),
                             "passed": result.returncode != 0 and batch["status"] == "failed" and "motion_control" not in batch["stages"]})
    sampling_contracts_ok = all(
        (s["planner"]["outcome"] == "success" and s["planner"].get("exit_code") == 0 and s["postprocess_status"] == "success")
        or (s["planner"]["outcome"] == "failed" and s["planner"].get("exit_code") != 0
            and bool(s["planner"].get("errors"))
            and all(error.get("code") == "NO_PATH" for error in s["planner"]["errors"])
            and s["postprocess_status"] is None)
        for s in sampling
    )
    passed = (all(c["exact_grid"] and c["hashes_match"] and c["pipeline"]["outcome"] == "success" and c["pipeline"].get("exit_code") == 0 and c["batch_status"] == "success" and c["motion_status"] == "pending" for c in checks)
              and all(a["outcome"] == "success" for a in acceptance)
              and all(f["passed"] for f in failures) and sampling_contracts_ok)
    report = {"created_at": datetime.now(timezone.utc).isoformat(), "python": platform.python_version(),
              "platform": platform.platform(), "versions": {"numpy": np.__version__, "opencv": cv2.__version__, "yaml": yaml.__version__},
              "status": "success" if passed else "failed", "public_maps": checks,
              "sampling_observations": sampling, "fake_ros2_acceptance": acceptance,
              "sampling_failure_contracts_ok": sampling_contracts_ok,
              "negative_cli_checks": failures,
              "scope": "offline software only; sampling NO_PATH is an observation, not a pass guarantee",
              "transient_artifacts": "TemporaryDirectory removed; no raw runs retained"}
    target = Path(args.output)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"Offline acceptance {report['status']}: {target}")
    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
