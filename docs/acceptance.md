# Acceptance Workflow

The repository provides a repeatable acceptance run for the complete software
pipeline and a Fake ROS2 motion run. It does not claim that simulated motion is
real-vehicle calibration.

## Automated acceptance

Run this from the project root:

```bash
python tools/acceptance.py
```

The default fixture is:

- map: `map_pipeline/data/input/01_scattered.png`
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

## Real-vehicle acceptance

Run the software inside the `turbopi` Docker container on the Raspberry Pi
host. Before driving, confirm that `/mecanum_chassis_node` subscribes to the
absolute `/cmd_vel` topic and isolate other velocity publishers. Use the
low-speed commands documented in `motion_control/README.md`.

Fill `acceptance/manual_record.json` after each check. Record the observed
direction, command, duration, battery, floor, payload, and result. Record the
empirical speed as an observation for those conditions; do not report it as a
universal centimetre-level calibration. The record must separately state
whether a zero `Twist` was observed after normal completion, an exception, and
Ctrl+C.
