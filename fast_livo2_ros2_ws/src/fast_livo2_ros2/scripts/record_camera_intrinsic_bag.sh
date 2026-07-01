#!/usr/bin/env bash
set -eo pipefail

IMAGE_TOPIC="${1:-/left_camera/image}"
OUT_ROOT="${2:-/home/czb/Tai Zhou/fast_livo2_ros2_ws/calibration/camera_intrinsic/bags}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${OUT_ROOT}/camera_intrinsic_${STAMP}"

export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
unset PYTHONHOME || true
unset AMENT_TRACE_SETUP_FILES || true
source /opt/ros/humble/setup.bash

mkdir -p "${OUT_ROOT}"
echo "Recording ${IMAGE_TOPIC} to ${OUT_DIR}"
ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/rosbag_camera_intrinsic_logs}" \
ros2 bag record "${IMAGE_TOPIC}" -o "${OUT_DIR}"
