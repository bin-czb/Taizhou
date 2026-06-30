#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
LIVO_WS="${ROOT_DIR}/fast_livo2_ros2_ws"
FASTLIO_WS="${ROOT_DIR}/fastlio_ros2_ws"

conda deactivate 2>/dev/null || true
unset PYTHONPATH PYTHONHOME
export ROS_LOG_DIR=/tmp/fastlio_external_imu_logs

source /opt/ros/humble/setup.bash
source "${LIVO_WS}/install/setup.bash"
source "${FASTLIO_WS}/install_clean/setup.bash"

ros2 launch fastlio2 lio_launch.py \
  config_path:="${FASTLIO_WS}/src/FASTLIO2_ROS2/fastlio2/config/lio_mid360_hwt905_external_imu.yaml" \
  use_rviz:=true

