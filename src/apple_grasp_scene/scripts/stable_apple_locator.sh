#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/song/workspace/ros2_kortex_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PYTHONPATH=/usr/lib/python3/dist-packages:${PYTHONPATH:-}
exec /usr/bin/python3 -s /home/song/桌面/apple_grasp_safety_ws/src/apple_grasp_scene/scripts/stable_apple_locator.py
