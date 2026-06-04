# Benchmarking

Run a dry benchmark to validate artifacts:

```bash
ros2 run dynamic_nav_benchmark experiment_runner \
  --config src/dynamic_nav_benchmark/experiments/tb3_mall.yaml \
  --dry-run
```

Run against a simulator:

```bash
ros2 launch dynamic_nav_bringup benchmark.launch.py \
  experiment:=/sim/src/dynamic_nav_benchmark/experiments/tb3_mall.yaml \
  backend:=classic world:=mall robot:=tb3_waffle_pi gui:=false
```

Run an included baseline:

```bash
ros2 launch dynamic_nav_bringup benchmark.launch.py \
  experiment:=/sim/src/dynamic_nav_benchmark/experiments/baseline_vfh_mall.yaml \
  backend:=classic world:=mall robot:=tb3_waffle_pi gui:=false
```

Baseline configs:

- `baseline_go_to_goal_mall.yaml`
- `baseline_bug2_mall.yaml`
- `baseline_vfh_mall.yaml`
- `baseline_follow_gap_mall.yaml`
- `baseline_astar_mall.yaml`
- `baseline_vfh_airport.yaml`
- `baseline_astar_airport.yaml`

For fair comparison with your own algorithm, copy a baseline config and replace
only `algorithm_launch`. Keep `world`, `seed`, `crowd_density`, `robot`,
`robot_start`, and `goals` unchanged.

Artifacts are written to `results/<run_id>`:

- `summary.json`
- `metrics.csv`
- `trajectory.csv`
- `plots/trajectory.png`
- `bags/` placeholder directory for rosbag output integration

Batch sweeps:

```bash
ros2 run dynamic_nav_benchmark batch_runner \
  --config src/dynamic_nav_benchmark/experiments/tb3_mall.yaml \
  --batch-id mall_smoke --dry-run
```

The runner writes `batch_summary.csv`, `batch_summary.json`, and sweep plots.
Set `record: true` in an experiment config to start `ros2 bag record` for the
configured sensor, command, goal, and benchmark event topics.
