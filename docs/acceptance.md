# Acceptance Workflow

The repository provides a repeatable acceptance run for the complete software
pipeline and a Fake ROS2 motion run. It does not claim that simulated motion is
real-vehicle calibration. Successful records are evidence of the version
that generated them, not a fresh verification of the current checkout.
Historical generated runs were moved out of the repository on 2026-11-14;
new runs are generated locally under the Git-ignored `runs/` directory.

## Automated acceptance

Latest detailed software scope and limitations:
[2026-11-14 offline verification](verification/2026-11-14-offline.md).
Install dependencies with `python -m pip install -r requirements-dev.txt`.
For all 12 public maps plus Fake ROS2 and negative CLI checks, run
`python tools/offline_acceptance.py --output docs/verification/latest-offline.json`.
This command removes transient run logs automatically and keeps a compact report.

Run this from the project root:

```bash
python tools/acceptance.py
```

The default software fixture is isolated from the public experiment maps:

- map: `tests/fixtures/01_scattered.png`
- start: `row=2,col=2`
- goal: `row=37,col=37`
- seed: `42`
- mode: `normal`

The command runs `map_pipeline`, `path_planner`, and `path_postprocess` through
their real JSON CLIs. It then runs the same motion executor against a Fake ROS2
graph containing `/mecanum_chassis_node` and no competing `/cmd_vel`
publishers.

For research-mode acceptance:

```bash
python tools/acceptance.py \
  --mode research \
  --algorithms astar,dijkstra \
  --seed 42
```

Each run writes:

- `manifest.json`: batch status, stage status, attempts, events, and artifacts
- `<module>/result.json`: versioned module contract
- `<module>/module.log`: stage trace
- `acceptance/result.json`: automated checks and Fake ROS2 command evidence
- `acceptance/manual_record.json`: real-vehicle observation template

Research acceptance checks multi-algorithm planning and post-processing, then
simulates the first successful post-processed comparison with Fake ROS2; the
top-level workflow itself skips its motion-control stage. A failed selected planner makes the overall planner
result fail, even when successful entries remain in the comparison array.

Software tests can also be run with `python -m unittest discover -s tests -q`.
Use a Python environment with `numpy`, `opencv-python`, and `pyyaml` installed.
On 2026-11-14 a project-local `.venv` was created with these dependencies.
All 136 software tests passed after the public-map replacement. All 12 public
maps matched their official grids cell-for-cell through the actual grid
preprocessor, and file hashes matched the manifest. One official task from
each family passed the real map/planner/postprocess CLIs in A*/Dijkstra
research mode without driving hardware.

## Real-vehicle acceptance

Run the software inside the `turbopi` Docker container on the Raspberry Pi
host. Before driving, confirm that `/mecanum_chassis_node` subscribes to the
absolute `/cmd_vel` topic and isolate other velocity publishers. Use the
execution and preflight procedure documented in `motion_control/README.md`,
with speed settings selected for low-speed verification. The default command
values are not a real-vehicle calibration.

Fill `acceptance/manual_record.json` after each check. Record the observed
direction, command, duration, battery, floor, payload, and result. Record the
empirical speed as an observation for those conditions; do not report it as a
universal centimetre-level calibration. The record must separately state
whether a zero `Twist` was observed after normal completion, an exception, and
Ctrl+C.

The current manual template provides a general exit zero-Twist field and a
Ctrl+C field, but no separate normal-completion and exception fields. Add
separate observations under `safety` (for example `normal_completion_stop`
and `exception_stop`) with command, observed zero Twist, wheel-stop behavior,
and outcome. Leave unchecked items pending until real-vehicle evidence exists.
