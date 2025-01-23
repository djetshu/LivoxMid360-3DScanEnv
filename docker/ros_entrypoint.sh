#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --  >> /root/.bashrc

# additional environment setup from /livox_mid_360_ws/install
source "/livox_mid_360/livox_mid_360_ws/install/setup.bash" -- || true  >> /root/.bashrc

# ----- source --------
cd /livox_mid_360/livox_mid_360_ws
source install/setup.bash

# Check if interactive and source bashrc for completions
if [[ $- == *i* ]]; then
    source ~/.bashrc
fi

echo -e "\033[0;32m==============FAST-LIO MID 360 ROS2 Docker Env Ready================\033[0m"

exec "$@"