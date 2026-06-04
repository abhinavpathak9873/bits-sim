# Algorithm API

Algorithms are black-box ROS nodes. They can be written in any language and do
not need to be Nav2 plugins.

Inputs:

- `/scan`: `sensor_msgs/LaserScan`
- `/odom`: `nav_msgs/Odometry`
- `/tf`, `/tf_static`
- `/map` when a map server or SLAM stack is launched
- `/goal_pose`: `geometry_msgs/PoseStamped`

Output:

- `/cmd_vel`: `geometry_msgs/Twist`

Gazebo goal visualization:

- `goal_marker` subscribes to `/goal_pose`
- It spawns/updates a visible Gazebo model named `dynamic_nav_goal_marker`
- The safe simulator starts it automatically when `GOAL_MARKER=true`

Manual launch:

```bash
ros2 launch dynamic_nav_benchmark goal_marker.launch.py
```

Namespaced robots should use the same contract under the robot namespace, for
example `/robot/cmd_vel`. Experiment configs can override topic names.

Benchmark events are published on `/benchmark/events` as simple string records:

- `run_start:<run_id>`
- `actor_spawned:<actor_name>`
- `goal_reached:<goal_name>`
- `timeout`
- `run_complete`

## Included Baselines

The benchmark package includes simple baseline algorithms that use the same
black-box API as external algorithms:

- `baseline_go_to_goal`: subscribes to `/goal_pose` and `/odom`, publishes `/cmd_vel`.
- `baseline_bug2`: subscribes to `/goal_pose`, `/odom`, `/scan`, publishes `/cmd_vel`.
- `baseline_vfh`: subscribes to `/goal_pose`, `/odom`, `/scan`, publishes `/cmd_vel`.
- `baseline_follow_gap`: subscribes to `/goal_pose`, `/odom`, `/scan`, publishes `/cmd_vel`.
- `baseline_astar`: subscribes to `/goal_pose`, `/odom`, `/scan`, publishes `/cmd_vel`.

Launch examples:

```bash
ros2 launch dynamic_nav_benchmark baseline_go_to_goal.launch.py
ros2 launch dynamic_nav_benchmark baseline_bug2.launch.py
ros2 launch dynamic_nav_benchmark baseline_vfh.launch.py
ros2 launch dynamic_nav_benchmark baseline_follow_gap.launch.py
ros2 launch dynamic_nav_benchmark baseline_astar.launch.py world:=mall
```

All baseline nodes accept topic parameters:

- `cmd_vel_topic`
- `goal_topic`
- `odom_topic`
- `scan_topic`

They also expose simple speed/controller parameters in their launch files.

`baseline_astar` also takes `world:=mall|airport` so it can load the matching
scenario obstacle boxes for its grid planner.
