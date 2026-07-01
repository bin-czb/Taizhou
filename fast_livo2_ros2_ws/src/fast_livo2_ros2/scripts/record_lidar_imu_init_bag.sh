#!/usr/bin/env bash
set -eo pipefail

OUTPUT_ROOT="${1:-${HOME}/fast_livo2_validation_bags}"
SESSION_NAME="${2:-lidar_imu_init_$(date +%Y%m%d_%H%M%S)}"
LIDAR_TOPIC="${LIDAR_TOPIC:-/livox/lidar}"
IMU_TOPIC="${IMU_TOPIC:-/imu/data}"

mkdir -p "${OUTPUT_ROOT}"
OUTPUT_PATH="${OUTPUT_ROOT}/${SESSION_NAME}"

echo "Recording LiDAR-IMU initialization rosbag:"
echo "  output: ${OUTPUT_PATH}"
echo "  lidar:  ${LIDAR_TOPIC}"
echo "  imu:    ${IMU_TOPIC}"
echo
echo "Motion guide:"
echo "  1. Keep the rig completely still for at least 5 seconds."
echo "  2. Rotate around roll, pitch, and yaw."
echo "  3. Move forward/back, left/right, and up/down."
echo "  4. Record 60-120 seconds in a feature-rich area."
echo
echo "Stop with Ctrl-C after the excitation is complete."

ros2 bag record \
  --output "${OUTPUT_PATH}" \
  "${LIDAR_TOPIC}" \
  "${IMU_TOPIC}" \
  /tf \
  /tf_static
