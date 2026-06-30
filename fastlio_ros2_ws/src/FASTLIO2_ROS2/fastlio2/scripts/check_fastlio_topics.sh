#!/usr/bin/env bash
set -euo pipefail

TIMEOUT_SEC="${1:-8}"

echo "Checking required FASTLIO2 input topics..."
ros2 topic info /livox/lidar || true
ros2 topic info /livox/imu || true
ros2 topic info /imu/data || true

echo
echo "Sampling rates for ${TIMEOUT_SEC}s each..."
echo "[/livox/lidar]"
timeout "${TIMEOUT_SEC}s" ros2 topic hz /livox/lidar --window 20 || true
echo "[/livox/imu]"
timeout "${TIMEOUT_SEC}s" ros2 topic hz /livox/imu --window 50 || true
echo "[/imu/data]"
timeout "${TIMEOUT_SEC}s" ros2 topic hz /imu/data --window 50 || true

echo
echo "FASTLIO2 output topics, if the node is running:"
ros2 topic info /fastlio2/lio_odom || true
ros2 topic info /fastlio2/lio_path || true
ros2 topic info /fastlio2/world_cloud || true
