#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
LIVO_WS="${ROOT_DIR}/fast_livo2_ros2_ws"

CAMERA_ID="${1:-0}"

conda deactivate 2>/dev/null || true
unset PYTHONPATH PYTHONHOME
export ROS_LOG_DIR=/tmp/usb_camera_logs

source /opt/ros/humble/setup.bash
source "${LIVO_WS}/install/setup.bash"

ros2 run fast_livo2_ros2 start_usb_camera.sh \
  "${CAMERA_ID}" 1280 720 30.0 /left_camera/image

