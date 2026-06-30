#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
LIVO_WS="${ROOT_DIR}/fast_livo2_ros2_ws"
FASTLIO_WS="${ROOT_DIR}/fastlio_ros2_ws"

conda deactivate 2>/dev/null || true
unset PYTHONPATH PYTHONHOME AMENT_TRACE_SETUP_FILES
export PATH=/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export ROS_LOG_DIR=/tmp/taizhou_build_logs

if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "[ERROR] ROS 2 Humble not found at /opt/ros/humble."
  exit 1
fi

echo "[1/2] Building fast_livo2_ros2_ws..."
cd "${LIVO_WS}"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

echo "[2/2] Building fastlio_ros2_ws..."
cd "${FASTLIO_WS}"
source /opt/ros/humble/setup.bash
source "${LIVO_WS}/install/setup.bash"
colcon --log-base log_clean build \
  --symlink-install \
  --build-base build_clean \
  --install-base install_clean \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

echo "Build complete."
echo "Source with:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source \"${LIVO_WS}/install/setup.bash\""
echo "  source \"${FASTLIO_WS}/install_clean/setup.bash\""

