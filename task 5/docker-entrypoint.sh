#!/bin/bash
set -e

# Source ROS 2 base
source /opt/ros/jazzy/setup.bash

# Source the built workspace overlay
if [ -f "/ros2_ws/install/setup.bash" ]; then
    source /ros2_ws/install/setup.bash
fi

# Run whatever CMD was passed (default: fleet.launch.py)
exec "$@"
