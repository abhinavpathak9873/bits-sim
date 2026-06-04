
# Use official ROS2 Humble base image
FROM osrf/ros:humble-desktop-full

# Build arguments
ARG WS_NAME=kuka_ws
ARG CONTAINER_NAME=container

# Set environment variables
ENV WS_NAME=${WS_NAME}
ENV CONTAINER_NAME=${CONTAINER_NAME}
ENV DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV ROS_DISTRO=humble

# Update and install essential packages
RUN apt-get update && apt-get install -y \
    wget \
    curl \
    git \
    vim \
    nano \
    build-essential \
    cmake \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3.10-venv \
    udev \
    && rm -rf /var/lib/apt/lists/*

# Install opencode
RUN curl -fsSL https://opencode.ai/install | bash

# Install dependencies for building librealsense from source
RUN apt-get update && apt-get install -y \
    libssl-dev \
    libusb-1.0-0-dev \
    libudev-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Build librealsense from source with RSUSB backend (no DKMS needed)
WORKDIR /usr/src
RUN git clone https://github.com/IntelRealSense/librealsense.git && \
    cd librealsense && \
    mkdir build && cd build && \
    cmake .. \
        -DFORCE_RSUSB_BACKEND=ON \
        -DBUILD_EXAMPLES=ON \
        -DBUILD_GRAPHICAL_EXAMPLES=ON \
        -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && \
    make install && \
    ldconfig

# Install ROS2 RealSense wrapper
RUN apt-get update && apt-get install -y \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description \
    && rm -rf /var/lib/apt/lists/*

# Install Gazebo packages
RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    && rm -rf /var/lib/apt/lists/*

# Install MoveIt2
RUN apt-get update && apt-get install -y \
    ros-humble-moveit \
    ros-humble-moveit-resources \
    ros-humble-moveit-visual-tools \
    ros-humble-moveit-servo \
    ros-humble-moveit-planners-ompl \
    && rm -rf /var/lib/apt/lists/*

# Install additional useful ROS2 packages
RUN apt-get update && apt-get install -y \
    ros-humble-rqt* \
    ros-humble-rviz2 \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

# Install OpenGL and display related packages
RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    mesa-utils \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# Create workspace directory
RUN mkdir -p /${WS_NAME}/src

# Set working directory
WORKDIR /${WS_NAME}

# Source ROS2 setup in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "if [ -f /${WS_NAME}/install/setup.bash ]; then source /${WS_NAME}/install/setup.bash; fi" >> ~/.bashrc

# Create smart entrypoint script that auto-installs udev rules
RUN printf '#!/bin/bash\nset -e\n\nGREEN="\033[0;32m"\nYELLOW="\033[1;33m"\nNC="\033[0m"\n\nif [ -d "/etc/udev/rules.d" ] && [ -w "/etc/udev/rules.d" ]; then\n    UDEV_RULES_FILE="/etc/udev/rules.d/99-realsense-libusb.rules"\n    SOURCE_RULES="/usr/src/librealsense/config/99-realsense-libusb.rules"\n    if [ ! -f "$UDEV_RULES_FILE" ]; then\n        echo -e "${YELLOW}Installing RealSense udev rules...${NC}"\n        cp "$SOURCE_RULES" "$UDEV_RULES_FILE"\n        if command -v udevadm >/dev/null 2>&1; then\n            udevadm control --reload-rules 2>/dev/null || true\n            udevadm trigger 2>/dev/null || true\n        fi\n        echo -e "${GREEN}RealSense udev rules installed!${NC}"\n    else\n        echo -e "${GREEN}RealSense udev rules already installed${NC}"\n    fi\nelse\n    echo -e "${YELLOW}Warning: /etc/udev/rules.d not mounted${NC}"\nfi\n\nsource /opt/ros/humble/setup.bash\n\nif [ -f /${WS_NAME}/install/setup.bash ]; then\n    source /${WS_NAME}/install/setup.bash\nfi\n\nexec "$@"\n' > /entrypoint.sh && chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
