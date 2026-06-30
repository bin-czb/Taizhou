#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
LIVO_WS="${ROOT_DIR}/fast_livo2_ros2_ws"

conda deactivate 2>/dev/null || true
unset PYTHONPATH PYTHONHOME
export ROS_LOG_DIR=/tmp/livox_mid360s_logs

source /opt/ros/humble/setup.bash
source "${LIVO_WS}/install/setup.bash"

ros2 launch livox_ros_driver2 msg_MID360s_launch.py

