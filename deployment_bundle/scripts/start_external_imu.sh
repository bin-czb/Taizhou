#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
LIVO_WS="${ROOT_DIR}/fast_livo2_ros2_ws"

PORT="${1:-/dev/ttyUSB0}"

conda deactivate 2>/dev/null || true
unset PYTHONPATH PYTHONHOME
export ROS_LOG_DIR=/tmp/hwt601_imu_logs

source /opt/ros/humble/setup.bash
source "${LIVO_WS}/install/setup.bash"

ros2 launch fast_livo2_ros2 hwt601_485_imu.launch.py \
  port:="${PORT}" \
  baudrate:=115200 \
  address:=80 \
  rate_hz:=150.0 \
  accel_range_g:=4.0

