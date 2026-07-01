#!/usr/bin/env bash
set -eo pipefail

DEVICE_ID="${1:-2}"
WIDTH="${2:-1280}"
HEIGHT="${3:-720}"
FREQUENCY="${4:-30.0}"
IMAGE_TOPIC="${5:-/left_camera/image}"

export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
unset PYTHONHOME || true
unset AMENT_TRACE_SETUP_FILES || true
source /opt/ros/humble/setup.bash

ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/cam2image_logs}" \
ros2 run image_tools cam2image --ros-args \
  -p device_id:="${DEVICE_ID}" \
  -p width:="${WIDTH}" \
  -p height:="${HEIGHT}" \
  -p frequency:="${FREQUENCY}" \
  -p show_camera:=false \
  -p frame_id:=left_camera \
  -r image:="${IMAGE_TOPIC}"
