#!/usr/bin/env bash
set -euo pipefail

sudo apt update
sudo apt install -y \
  build-essential cmake git wget curl \
  python3-pip python3-serial python3-yaml python3-numpy python3-opencv \
  python3-colcon-common-extensions python3-rosdep \
  v4l-utils can-utils zstd pigz \
  libpcl-dev libeigen3-dev libyaml-cpp-dev libgtsam-dev \
  ros-humble-desktop \
  ros-humble-rviz2 \
  ros-humble-pcl-conversions \
  ros-humble-tf2-ros \
  ros-humble-message-filters \
  ros-humble-rosbag2 \
  ros-humble-rosbag2-storage-default-plugins \
  ros-humble-cv-bridge \
  ros-humble-image-transport

echo "APT dependencies installed."
echo "Install Livox-SDK2 and Sophus manually if they are not already installed."

