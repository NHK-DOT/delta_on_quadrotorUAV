FROM ros:humble-desktop

# Install ROS 2 and system dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    ros-humble-tf2-geometry-msgs \
    python3-colcon-common-extensions \
    python3-pip \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Install GeographicLib datasets (required by MAVROS)
RUN wget -qO- https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh | bash

# Set up workspace directory
WORKDIR /workspace/uav_delta_capture
