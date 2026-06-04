# Dynamic TurtleBot Navigation Simulation Environment

Research workspace for benchmarking navigation algorithms in dynamic indoor
environments with ROS 2 Humble. The workspace provides mall and airport
scenarios, deterministic dynamic obstacles, benchmark tooling, and launch
surfaces for official TurtleBot3 and TurtleBot4 simulation packages.

## Packages

- `dynamic_nav_worlds`: worlds, scenario configs, maps, and shared assets.
- `dynamic_nav_bringup`: simulator launch files and dependency checks.
- `dynamic_nav_actors`: deterministic dynamic obstacle manager.
- `dynamic_nav_benchmark`: experiment runner, metrics, plots, and dummy nodes.
- `dynamic_nav_docs`: setup and authoring documentation.

## Quick Start

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# Classic Gazebo + TurtleBot3, requires official TurtleBot3 simulation packages.
ros2 launch dynamic_nav_bringup simulation.launch.py \
  backend:=classic world:=mall robot:=tb3_burger crowd_density:=low

# Safer local GUI runner for this container/display setup.
DISPLAY=:1 GUI=true DYNAMIC_OBSTACLES=false \
  /sim/install/dynamic_nav_bringup/share/dynamic_nav_bringup/scripts/safe_classic_sim.sh

# Benchmark dry run that writes output artifacts without launching a simulator.
ros2 run dynamic_nav_benchmark experiment_runner \
  --config src/dynamic_nav_benchmark/experiments/tb3_mall.yaml --dry-run
```

See `src/dynamic_nav_docs/docs/setup.md` for TurtleBot package prerequisites.
