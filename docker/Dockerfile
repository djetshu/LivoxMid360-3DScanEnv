# Start from the ROS 2 Humble Full Desktop image
FROM osrf/ros:humble-desktop-full

# Install basic utilities
RUN apt update && apt install -y \
    nano \
    vim \
    gedit \
    tmux \
    pip \
    libsdl1.2-dev \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-xacro \
    ros-humble-teleop-twist-keyboard \
    ros-humble-controller-manager \
    ros-humble-rqt* \
    && rm -rf /var/lib/apt/lists/*

# Install Gazebo and missing ROS packages
RUN apt update && apt install -y \
    gazebo \
    libgazebo11 \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-rviz2 \
    ros-humble-rqt \
    ros-humble-rqt-graph \
    ros-humble-gazebo-ros2-control \
    ros-humble-joint-state-publisher-gui \
    liburdfdom-tools \
    ros-humble-ament-cmake \
    && rm -rf /var/lib/apt/lists/*

# Set up environment variables for Gazebo
ENV GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:$GAZEBO_MODEL_PATH \
    GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:$GAZEBO_PLUGIN_PATH \
    GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11:$GAZEBO_RESOURCE_PATH \
    LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:$LD_LIBRARY_PATH

# Run rosdep update
RUN rosdep update

# Set up
RUN apt-get update

# Update bashrc
RUN echo 'source "/opt/ros/$ROS_DISTRO/setup.bash"' >> ~/.bashrc

# Set the default working directory
WORKDIR /livox_mid_360/

# Copy the ros_entrypoint.sh script and set permissions
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

# Set the entrypoint and default command
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash", "-i"]