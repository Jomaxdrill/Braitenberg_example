FROM osrf/ros:galactic-desktop

# Install basic dependencies
RUN apt-get update && apt-get install -y \
    python3-rosdep \
    python3-colcon-common-extensions \
    ros-galactic-tf2 \
    ros-galactic-urdf \
    ros-galactic-sensor-msgs \
    ros-galactic-nav-msgs \
    ros-galactic-geometry-msgs \
    ros-galactic-rclcpp \
    && rm -rf /var/lib/apt/lists/*

# Create workspace directory
RUN mkdir -p /ROS_WS/src
WORKDIR /ROS_WS

# Copy your ROS packages
COPY braitenberg_vehicle/ ./src/braitenberg_vehicle/
COPY braitenberg_vehicle_description/ ./src/braitenberg_vehicle_description/


# Source ROS and workspace in .bashrc
RUN echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc

# X11 forwarding for Gazebo GUI
ENV DISPLAY=:0
ENV LIBGL_ALWAYS_INDIRECT=1