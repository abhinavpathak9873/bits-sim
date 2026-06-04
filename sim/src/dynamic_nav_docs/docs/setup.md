# Setup

This workspace targets ROS 2 Humble on Ubuntu 22.04 with Gazebo Classic 11
and the ROS Gazebo bridge packages.

Base packages expected on this machine:

```bash
sudo apt install \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-plugins \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  python3-colcon-common-extensions
```

Official TurtleBot packages are intentionally not vendored. Install/source them
before launching those robot backends.

TurtleBot3 Classic backend:

```bash
# If binaries are unavailable for your apt source, build from source.
mkdir -p ~/tb3_ws/src
cd ~/tb3_ws/src
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
cd ~/tb3_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

TurtleBot4 Ignition backend:

```bash
# Prefer distro packages when available from your ROS apt source.
# Otherwise follow the official TurtleBot4 Humble simulator instructions.
ros2 pkg prefix turtlebot4_ignition_bringup
```

Workspace build:

```bash
cd /sim
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Launch examples:

```bash
ros2 launch dynamic_nav_bringup simulation.launch.py backend:=classic world:=mall robot:=tb3_waffle_pi
ros2 launch dynamic_nav_bringup simulation.launch.py backend:=ignition world:=airport robot:=tb4_standard
```
