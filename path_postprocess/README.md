# path_postprocess

Converts a successful planner result into a validated, executable path:

1. Smooths the raw grid path with a replaceable natural cubic spline.
2. Samples and rounds the curve to grid coordinates.
3. Rechecks the continuous sampled curve and its grid projection for bounds and obstacles.
4. Falls back to the validated raw path when smoothing fails.
5. Splits long connections and emits controller-facing JSON segments.

## API

```python
from path_postprocess import process_path

result = process_path(
    grid,
    raw_path,
    grid_size_m=0.3,
    parameters={"max_segment_length_grid": 2.0},
)
```

The successful result stores `raw_path`, `smoothed_path`, `segments`,
`smoothing`, and `metrics` under `outputs`. A smoothing fallback remains a
successful post-processing result and records `outputs.smoothing.fallback_reason`.

The `smoothed_path` payload stores the validated floating-point samples. The
executable `segments` payload stores integer grid endpoints for the current
controller contract.

The smoother boundary is `PathSmoother.smooth(path, sample_spacing_grid)`.
`NaturalCubicSplineSmoother` is the first implementation; another smoother can
be passed to `process_path(..., smoother=...)` without changing the result
contract.

## CLI

```bash
python -m path_postprocess.run_postprocess \
  --map-result runs/map-001/map_pipeline/result.json \
  --planner-result runs/planner-001/path_planner/result.json \
  --run-root runs \
  --run-id postprocess-001
```

The CLI writes `runs/<run-id>/path_postprocess/result.json` and preserves a
structured failure result when either upstream result is missing or failed.
