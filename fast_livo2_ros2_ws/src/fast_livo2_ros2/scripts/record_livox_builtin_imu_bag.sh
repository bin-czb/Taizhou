#!/usr/bin/env bash
set -eo pipefail

OUTPUT_ROOT="${1:-${HOME}/fast_livo2_validation_bags}"
SESSION_NAME="${2:-livox_builtin_imu_$(date +%Y%m%d_%H%M%S)}"
LIDAR_TOPIC="${LIDAR_TOPIC:-/livox/lidar}"
LIVOX_IMU_TOPIC="${LIVOX_IMU_TOPIC:-/livox/imu}"

OUTPUT_ROOT="${OUTPUT_ROOT/#\~/${HOME}}"
OUTPUT_PATH="${OUTPUT_ROOT}/${SESSION_NAME}"
mkdir -p "${OUTPUT_ROOT}"

echo "Recording Livox Mid360 built-in IMU diagnostic bag:"
echo "  output:    ${OUTPUT_PATH}"
echo "  lidar:     ${LIDAR_TOPIC}"
echo "  livox imu: ${LIVOX_IMU_TOPIC}"
echo
echo "Checking topics..."
ros2 topic info "${LIDAR_TOPIC}" || true
ros2 topic info "${LIVOX_IMU_TOPIC}" || true
echo
echo "Quick rate check. If /livox/imu shows no average rate, the topic exists but no IMU samples are arriving."
timeout 6s ros2 topic hz "${LIDAR_TOPIC}" || true
timeout 6s ros2 topic hz "${LIVOX_IMU_TOPIC}" || true
echo
echo "Stop with Ctrl-C after 20-60 seconds."

ros2 bag record --storage sqlite3 --output "${OUTPUT_PATH}" "${LIDAR_TOPIC}" "${LIVOX_IMU_TOPIC}"
