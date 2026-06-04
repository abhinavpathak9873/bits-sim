# Dynamic TurtleBot Navigation Simulation Environment

ROS 2 Humble workspace for testing TurtleBot navigation algorithms in dynamic
indoor environments. It includes mall and airport scenarios, moving pedestrians,
service carts, static clutter, repeatable seeds, and benchmark tooling.

The most reliable local path right now is the Gazebo Classic safe runner for
TurtleBot3. TurtleBot4/Ignition support is scaffolded through the official
TurtleBot4 simulation packages.

Agent and contributor coding rules live in `AGENTS.md`.

## Packages

- `dynamic_nav_worlds`: world files, scenario YAML, maps, and shared assets.
- `dynamic_nav_bringup`: launch files and the safe Gazebo Classic runner.
- `dynamic_nav_actors`: ROS actor manager fallback.
- `dynamic_nav_gazebo_plugins`: smooth in-Gazebo scripted actor motion.
- `dynamic_nav_benchmark`: experiment runner, dummy algorithm, metrics, plots.
- `dynamic_nav_docs`: setup, scenario authoring, algorithm API, benchmarking docs.

## Build

```bash
cd /sim
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run The GUI

Mall:

```bash
DISPLAY=:0 GUI=true WORLD=mall ROBOT=tb3_waffle_pi HUMAN_SCALE=0.9 \
  DYNAMIC_OBSTACLES=true CROWD_DENSITY=low SPEED_PROFILE=medium MAX_ACTORS=8 \
  /sim/install/dynamic_nav_bringup/share/dynamic_nav_bringup/scripts/safe_classic_sim.sh
```

Airport:

```bash
DISPLAY=:0 GUI=true WORLD=airport ROBOT=tb3_waffle_pi HUMAN_SCALE=0.9 \
  DYNAMIC_OBSTACLES=true CROWD_DENSITY=low SPEED_PROFILE=medium MAX_ACTORS=8 \
  /sim/install/dynamic_nav_bringup/share/dynamic_nav_bringup/scripts/safe_classic_sim.sh
```

Static world only:

```bash
DISPLAY=:0 GUI=true DYNAMIC_OBSTACLES=false \
  /sim/install/dynamic_nav_bringup/share/dynamic_nav_bringup/scripts/safe_classic_sim.sh
```

Stop the simulation with `Ctrl-C` in the terminal running the script.

When `GOAL_MARKER=true` the simulator also starts a goal marker node. Every
`/goal_pose` message creates a bright green/yellow marker in Gazebo at the target
pose, so you can see where the robot is trying to go.

## Recommended Settings

- `ROBOT=tb3_waffle_pi` looks better in the scene than `tb3_burger`.
- `HUMAN_SCALE=0.9` keeps people human-ish while making the visual scale nicer.
- `MAX_ACTORS=6-12` is a good GUI range.
- `CROWD_DENSITY=low` or `medium` is best for visual debugging.
- `CROWD_DENSITY=high` or `stress` is for benchmark stress tests.
- `SPEED_PROFILE=medium` is the default nice-looking speed.

TurtleBot3 Burger is genuinely tiny in real life. Its local Gazebo body model is
roughly `0.14 m` wide, so it will look very small next to human-scale actors.

## Runtime Controls

- `WORLD=mall|airport`
- `ROBOT=tb3_burger|tb3_waffle|tb3_waffle_pi`
- `GUI=true|false`
- `DYNAMIC_OBSTACLES=true|false`
- `CROWD_DENSITY=low|medium|high|stress`
- `SPEED_PROFILE=static|low|medium|mid|high`
- `MAX_ACTORS=8`
- `HUMAN_SCALE=1.0`
- `SEED=1`
- `ROBOT_X`, `ROBOT_Y`, `ROBOT_YAW`
- `ACTOR_CONTROL=plugin`
- `GOAL_MARKER=true|false`

Example custom robot start:

```bash
ROBOT_X=1.0 ROBOT_Y=-2.0 ROBOT_YAW=1.57 DISPLAY=:0 GUI=true \
  WORLD=mall DYNAMIC_OBSTACLES=true \
  /sim/install/dynamic_nav_bringup/share/dynamic_nav_bringup/scripts/safe_classic_sim.sh
```

## Actor Behavior

Actors are generated deterministically from the scenario seed.

- People and carts move smoothly inside Gazebo through `dynamic_nav_gazebo_plugins`.
- People vary in height, width, color, walking speed, sway, and bobbing motion.
- Actors pass through each other to avoid ugly crowd clumps.
- Actors steer around the robot.
- Actors bounce off configured static obstacles.
- Mall obstacles include kiosks, storefront blocks, food court, and planters.
- Airport obstacles include check-in islands, security lane, and gate seating.

## Moving The Robot

The robot listens on `/cmd_vel`.

Drive forward:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.15}, angular: {z: 0.0}}"
```

Spin:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.6}}"
```

Stop:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

Useful topics:

- `/scan`
- `/odom`
- `/tf`
- `/tf_static`
- `/cmd_vel`

## Benchmark Dry Run

```bash
ros2 run dynamic_nav_benchmark experiment_runner \
  --config src/dynamic_nav_benchmark/experiments/tb3_mall.yaml --dry-run
```

## Baseline Algorithms

The workspace includes five simple black-box navigation baselines. They all use
the same topics your own algorithm should use: `/goal_pose`, `/odom`, `/scan`,
and `/cmd_vel`.

- `baseline_go_to_goal`: proportional go-to-goal controller. Fast and simple, no obstacle avoidance.
- `baseline_bug2`: bug-style controller. Goes to the goal, then follows a wall when blocked.
- `baseline_vfh`: lightweight vector-field histogram style controller using lidar repulsion.
- `baseline_follow_gap`: follow-the-gap lidar controller. Picks the largest open lidar gap.
- `baseline_astar`: grid A* global planner over known static scenario obstacles, then waypoint following.

### Watch A Baseline In The GUI

Use three terminals.

Terminal 1, start the simulator:

```bash
cd /sim
source /opt/ros/humble/setup.bash
source install/setup.bash

DISPLAY=:0 GUI=true WORLD=mall ROBOT=tb3_waffle_pi HUMAN_SCALE=0.9 \
  DYNAMIC_OBSTACLES=true CROWD_DENSITY=low SPEED_PROFILE=medium MAX_ACTORS=8 \
  /sim/install/dynamic_nav_bringup/share/dynamic_nav_bringup/scripts/safe_classic_sim.sh
```

Terminal 2, start exactly one baseline algorithm:

```bash
cd /sim
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch dynamic_nav_benchmark baseline_vfh.launch.py
```

Other baseline launch choices:

```bash
ros2 launch dynamic_nav_benchmark baseline_go_to_goal.launch.py
ros2 launch dynamic_nav_benchmark baseline_bug2.launch.py
ros2 launch dynamic_nav_benchmark baseline_vfh.launch.py
ros2 launch dynamic_nav_benchmark baseline_follow_gap.launch.py
ros2 launch dynamic_nav_benchmark baseline_astar.launch.py world:=mall
```

For airport, use:

```bash
ros2 launch dynamic_nav_benchmark baseline_astar.launch.py world:=airport
```

Terminal 3, send a mall goal:

```bash
cd /sim
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 10.5, y: 6.0, z: 0.0}, orientation: {w: 1.0}}}"
```

A bright marker named `dynamic_nav_goal_marker` should appear at that point in
Gazebo.

Airport goal:

```bash
cd /sim
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 12.0, y: 5.5, z: 0.0}, orientation: {w: 1.0}}}"
```

Only run one baseline at a time because they all publish `/cmd_vel`.

If you started the simulator before goal markers were added, run this once in
another sourced terminal:

```bash
ros2 launch dynamic_nav_benchmark goal_marker.launch.py
```

### Baseline Parameters

Common parameters:

- `cmd_vel_topic=/cmd_vel`
- `goal_topic=/goal_pose`
- `odom_topic=/odom`
- `scan_topic=/scan`
- `max_linear_speed`
- `max_angular_speed`
- `goal_tolerance_m`

Algorithm-specific parameters:

- `baseline_bug2`: `obstacle_enter_m`, `obstacle_exit_m`, `desired_wall_distance_m`
- `baseline_vfh`: `influence_distance_m`, `front_stop_m`, `repulsive_gain`
- `baseline_follow_gap`: `free_distance_m`, `bubble_radius_rad`, `heading_weight`
- `baseline_astar`: `world`, `grid_resolution_m`, `obstacle_inflation_m`, `waypoint_tolerance_m`, `front_stop_m`

Edit the launch files in `src/dynamic_nav_benchmark/launch/` to tune defaults.

### Benchmark A Baseline

Benchmark one baseline config:

```bash
ros2 launch dynamic_nav_bringup benchmark.launch.py \
  experiment:=/sim/src/dynamic_nav_benchmark/experiments/baseline_vfh_mall.yaml \
  backend:=classic world:=mall robot:=tb3_waffle_pi gui:=false
```

Available baseline experiment configs:

- `src/dynamic_nav_benchmark/experiments/baseline_go_to_goal_mall.yaml`
- `src/dynamic_nav_benchmark/experiments/baseline_bug2_mall.yaml`
- `src/dynamic_nav_benchmark/experiments/baseline_vfh_mall.yaml`
- `src/dynamic_nav_benchmark/experiments/baseline_follow_gap_mall.yaml`
- `src/dynamic_nav_benchmark/experiments/baseline_astar_mall.yaml`
- `src/dynamic_nav_benchmark/experiments/baseline_vfh_airport.yaml`
- `src/dynamic_nav_benchmark/experiments/baseline_astar_airport.yaml`

To benchmark your own algorithm, copy one of those YAML files and replace
`algorithm_launch` with your launch command. Keep the same world, seed, crowd
density, robot, start, and goal when comparing algorithms.

## Integrating Your Own Algorithm

Your planner/controller should behave like a black-box ROS node. It does not
need to be a Nav2 plugin.

### Required Inputs

Subscribe to the topics you need from this contract:

- `/goal_pose`: `geometry_msgs/msg/PoseStamped`
- `/odom`: `nav_msgs/msg/Odometry`
- `/scan`: `sensor_msgs/msg/LaserScan`
- `/tf`: `tf2_msgs/msg/TFMessage`
- `/tf_static`: `tf2_msgs/msg/TFMessage`
- `/map`: `nav_msgs/msg/OccupancyGrid`, only when you launch a map source

Most baseline-style algorithms only need `/goal_pose`, `/odom`, and `/scan`.

Topic meanings:

- `/goal_pose` is the current navigation target. The safe simulator also draws this as a Gazebo marker.
- `/odom` gives the robot pose and velocity in the odometry frame.
- `/scan` gives lidar obstacle ranges.
- `/tf` and `/tf_static` provide frame transforms if your algorithm uses TF.

### Required Output

Publish velocity commands to:

- `/cmd_vel`: `geometry_msgs/msg/Twist`

Use:

- `linear.x` for forward/backward velocity in meters per second.
- `angular.z` for yaw rate in radians per second.

Do not run multiple algorithms that publish `/cmd_vel` at the same time unless
you intentionally add a command multiplexer.

### Frames

Use these assumptions unless your launch file remaps them:

- Goal frame: `map`
- Robot odometry frame: from `/odom`
- Robot base frame: available through `/tf`, typically `base_footprint` or `base_link`
- Lidar frame: available through `/tf`

The default examples publish goals with `header.frame_id: map`. If your
algorithm only works in odom, transform the goal through TF or use the current
sim setup consistently for all compared algorithms.

### Launch Contract

Your algorithm should be launchable with one command, for example:

```bash
ros2 launch my_nav_package my_algorithm.launch.py
```

That launch file should start exactly the nodes needed by your algorithm and
should publish only one final command stream to `/cmd_vel`.

Recommended configurable parameters:

- `cmd_vel_topic`
- `goal_topic`
- `odom_topic`
- `scan_topic`
- `max_linear_speed`
- `max_angular_speed`

### Manual Test Workflow

Terminal 1, start the sim:

```bash
cd /sim
source /opt/ros/humble/setup.bash
source install/setup.bash

DISPLAY=:0 GUI=true WORLD=mall ROBOT=tb3_waffle_pi HUMAN_SCALE=0.9 \
  DYNAMIC_OBSTACLES=true CROWD_DENSITY=low SPEED_PROFILE=medium MAX_ACTORS=8 \
  /sim/install/dynamic_nav_bringup/share/dynamic_nav_bringup/scripts/safe_classic_sim.sh
```

Terminal 2, start your algorithm:

```bash
cd /sim
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch my_nav_package my_algorithm.launch.py
```

Terminal 3, send a goal:

```bash
cd /sim
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 10.5, y: 6.0, z: 0.0}, orientation: {w: 1.0}}}"
```

### Benchmark Config

Create a YAML file by copying one baseline experiment:

```bash
cp src/dynamic_nav_benchmark/experiments/baseline_vfh_mall.yaml \
  src/dynamic_nav_benchmark/experiments/my_algorithm_mall.yaml
```

Change only the identity and launch command first:

```yaml
run_id: my_algorithm_mall
algorithm_launch: ros2 launch my_nav_package my_algorithm.launch.py
```

Keep these fields unchanged when comparing against baselines:

- `world`
- `backend`
- `robot`
- `seed`
- `crowd_density`
- `dynamic_obstacles`
- `robot_start`
- `goals`
- `timeout_s`
- `goal_tolerance_m`

Run the benchmark:

```bash
ros2 launch dynamic_nav_bringup benchmark.launch.py \
  experiment:=/sim/src/dynamic_nav_benchmark/experiments/my_algorithm_mall.yaml \
  backend:=classic world:=mall robot:=tb3_waffle_pi gui:=false
```

### Integration Checklist

- Your node receives `/goal_pose`.
- Your node receives `/odom`.
- Your node receives `/scan` if it does obstacle avoidance.
- Your node publishes `/cmd_vel`.
- Only one `/cmd_vel` publisher is active during a run.
- The robot stops when it reaches the goal tolerance.
- Your launch command works from a freshly sourced `/sim` terminal.
- Your benchmark YAML uses the same seed/world/goal as the baseline you compare against.

Example experiment configs:

- `src/dynamic_nav_benchmark/experiments/tb3_mall.yaml`
- `src/dynamic_nav_benchmark/experiments/tb3_airport.yaml`
- `src/dynamic_nav_benchmark/experiments/tb4_mall.yaml`
- `src/dynamic_nav_benchmark/experiments/tb4_airport.yaml`

## ROS Launch Path

Classic Gazebo + TurtleBot3 launch path:

```bash
ros2 launch dynamic_nav_bringup simulation.launch.py \
  backend:=classic world:=mall robot:=tb3_burger crowd_density:=low
```

The safe runner is usually better for this container/display setup.

## Troubleshooting

Check for leftover Gazebo processes:

```bash
pgrep -af 'gzserver|gzclient|safe_classic_sim|actor_manager|spawn_entity|gz model' || true
```

Kill leftover simulation processes only if needed:

```bash
pkill -f safe_classic_sim.sh || true
pkill -f gzserver || true
pkill -f gzclient || true
```

If the GUI does not appear, make sure `DISPLAY=:0` is correct:

```bash
DISPLAY=:0 gazebo
```

Gazebo Classic may print `sequence size exceeds remaining buffer`. In current
testing this warning appears with TurtleBot camera/sensor messages but does not
prevent the world, robot, actors, `/scan`, `/odom`, or `/cmd_vel` from working.

See `src/dynamic_nav_docs/docs/setup.md` for dependency notes.
