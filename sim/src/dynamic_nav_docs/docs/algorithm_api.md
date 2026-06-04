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

Namespaced robots should use the same contract under the robot namespace, for
example `/robot/cmd_vel`. Experiment configs can override topic names.

Benchmark events are published on `/benchmark/events` as simple string records:

- `run_start:<run_id>`
- `actor_spawned:<actor_name>`
- `goal_reached:<goal_name>`
- `timeout`
- `run_complete`
