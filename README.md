# ROS2 Humble Docker Setup

A comprehensive Docker environment for ROS2 Humble with Intel RealSense support, GPU passthrough, and robotics tools.

## Features

 - **ROS2 Humble Desktop Full** - Complete ROS2 installation  
 - **Intel RealSense Support** - SDK and ROS2 wrapper pre-installed  
 - **NVIDIA GPU Passthrough** - Automatic GPU detection and configuration  
 - **USB Passthrough** - Direct access to RealSense cameras and other USB devices  
 - **Display Passthrough** - Run GUI applications (RViz2, Gazebo, rqt)  
 - **Host Networking** - Seamless ROS2 node discovery  
 - **Persistent Storage** - Your workspace is saved on the host  
 - **Gazebo Simulation** - Pre-installed and configured  
 - **MoveIt2** - Motion planning framework included  

## Prerequisites

### Required
- Docker (version 20.10+)
- Docker Compose (version 1.29+)

### Optional (for GPU support)
- NVIDIA GPU with drivers installed
- nvidia-docker2 runtime

To install nvidia-docker2:
```bash
sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker
```

## Quick Start

### 1. Clone or Download This Setup

Place all the files in a directory:
```
ros2-docker/
├── Dockerfile
├── docker-compose.yml
├── build.sh
├── start.sh
└── README.md
```

### 2. Build the Docker Image

```bash
chmod +x *.sh
./build.sh
```

This script will:
- Check for Docker and Docker Compose
- Detect NVIDIA GPU if available
- Create workspace directories
- Build the Docker image

### 3. Start the Container

```bash
./start.sh
```


## Usage

### Testing RealSense Camera

Inside the container:
```bash
# Check if camera is detected
rs-enumerate-devices

# View camera stream
realsense-viewer

# Launch ROS2 node
ros2 launch realsense2_camera rs_launch.py
```

### Testing GPU (NVIDIA)

```bash
# Check GPU
nvidia-smi

# Test OpenGL
glxinfo | grep "OpenGL"
```

### Testing Gazebo

```bash
# Launch empty world
gazebo

# Or with ROS2 integration
ros2 launch gazebo_ros gazebo.launch.py
```

### Testing MoveIt2

```bash
# Launch MoveIt2 demo
ros2 launch moveit2_tutorials demo.launch.py
```

### Testing RViz2

```bash
rviz2
```

### Building The ROS2 Workspace

```bash
# Navigate to workspace
cd /ros2_ws

# Create a package (example)
cd src
ros2 pkg create --build-type ament_cmake my_robot_pkg

# Build workspace
cd /ros2_ws
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

## Directory Structure

```
Host Machine:
├── ros2_ws/              # Your ROS2 workspace (persistent)
│   ├── src/              # Source packages
│   ├── build/            # Build files
│   ├── install/          # Installed packages
│   └── log/              # Build logs
└── config/
    └── .bashrc           # Custom bash configuration

Inside Container:
└── /ros2_ws/             # Mounted workspace
```

## Helpful Aliases

The container includes these pre-configured aliases:

- `cb` - Build workspace (`colcon build --symlink-install`)
- `cs` - Source workspace
- `ws` - Go to workspace directory
- `rt` - List ROS2 topics
- `rn` - List ROS2 nodes
- `rs` - List ROS2 services

# Troubleshooting

### Display Issues

If GUI applications don't start:
```bash
# On host machine
xhost +local:docker
```

### RealSense Not Detected

```bash
# Check USB connections
lsusb | grep Intel

# Inside container
rs-enumerate-devices
```

### GPU Not Working

1. Verify NVIDIA drivers on host:
   ```bash
   nvidia-smi
   ```

2. Check nvidia-docker2:
   ```bash
   docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi
   ```

3. Ensure Docker Compose version supports GPU (1.29+):
   ```bash
   docker-compose version
   ```

### Permission Issues

If you encounter permission issues with the workspace:
```bash
# On host machine
sudo chown -R $USER:$USER ros2_ws/
```

## Scripts Reference

- **build.sh** - Initial setup and image building
- **start.sh** - Start the container (detached mode)
- **shell.sh** - Enter the container shell
- **stop.sh** - Stop the container

## Container Management

```bash
# Start container
./start.sh

# View logs
docker-compose logs -f

# Restart container
docker-compose restart

# Remove container (keeps image)
docker-compose down

# Remove everything including volumes
docker-compose down -v
```

## Customization

### Add More ROS2 Packages

Edit `Dockerfile` and add packages before the build:
```dockerfile
RUN apt-get update && apt-get install -y \
    ros-humble-your-package \
    && rm -rf /var/lib/apt/lists/*
```

Then rebuild:
```bash
docker-compose build --no-cache
```

### Add More USB Devices

Edit `docker-compose.yml` under `devices:` section:
```yaml
devices:
  - /dev/ttyUSB0:/dev/ttyUSB0  # Serial device
  - /dev/video0:/dev/video0     # Webcam
```

## Notes

- The workspace (`ros2_ws/`) persists on your host machine
- Container runs in privileged mode for full USB/device access
- All data in `/ros2_ws` is preserved between container restarts
- GPU support is automatic if NVIDIA GPU and drivers are detected
# kuka_iiwa_robox
# kuka_iiwa_robox
# kuka_iiwa_robox
# kuka_iiwa_robox
