#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --  >> /root/.bashrc

# additional environment setup from /base_lunar_builder_ws/install
# source "/lunar_builder_2025/base_lunar_builder_ws/install/setup.bash" -- || true  >> /root/.bashrc

# ----- Livox-SDK2 --------
cd /livox_mid_360/livox_mid_360_ws/src/Livox-SDK2/build
cmake .. && make -j
sudo make install

# ----- FAST-LIO --------
cd /livox_mid_360/livox_mid_360_ws
rosdep install --from-paths src --ignore-src -y
#colcon build

# ----- livox-ros-driver2 -------
cd /livox_mid_360/livox_mid_360_ws/src/livox_ros_driver2
source /opt/ros/humble/setup.sh
./build.sh humble

# ----- source --------
cd /livox_mid_360/livox_mid_360_ws
source install/setup.bash

# Check if interactive and source bashrc for completions
if [[ $- == *i* ]]; then
    source ~/.bashrc
fi

exec "$@"