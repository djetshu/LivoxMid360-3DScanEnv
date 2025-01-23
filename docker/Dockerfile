# Start from the ROS 2 Humble Full Desktop image
FROM osrf/ros:humble-desktop-full

# Install basic utilities
RUN apt update && apt install -y \
    nano \
    vim \
    gedit \
    tmux \
    net-tools \
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

# Copy livox_mid_360 and pre-build it
COPY livox_mid_360/ /livox_mid_360/livox_mid_360_ws/

# Set up
RUN apt-get update

# Change working directory to Livox-SDK2 build folder
WORKDIR /livox_mid_360/livox_mid_360_ws/src/Livox-SDK2/

# Build and install Livox-SDK2
RUN mkdir -p build && cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install

# Install FAST-LIO
WORKDIR /livox_mid_360/livox_mid_360_ws/
RUN rosdep install --from-paths src --ignore-src -y

# Install livox-ros-driver2
WORKDIR /livox_mid_360/livox_mid_360_ws/src/livox_ros_driver2/

# Source ROS environment and build the base workspace
RUN bash -c "source /opt/ros/humble/setup.bash && \
    ./build.sh humble"

# Update bashrc
RUN echo 'source "/opt/ros/$ROS_DISTRO/setup.bash"' >> ~/.bashrc

# Set the default working directory
WORKDIR /livox_mid_360/livox_mid_360_ws/

# Copy the ros_entrypoint.sh script and set permissions
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

# Set the entrypoint and default command
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash", "-i"]