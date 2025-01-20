#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --  >> /root/.bashrc

# additional environment setup from /base_lunar_builder_ws/install
# source "/lunar_builder_2025/base_lunar_builder_ws/install/setup.bash" -- || true  >> /root/.bashrc

# Check if interactive and source bashrc for completions
if [[ $- == *i* ]]; then
    source ~/.bashrc
fi

exec "$@"