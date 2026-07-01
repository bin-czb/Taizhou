#!/bin/bash
set -e

source /opt/ros/noetic/setup.bash
if [ -f /home/catkin_ws/devel/setup.bash ]; then
  source /home/catkin_ws/devel/setup.bash
fi

echo "================ LI-Init Noetic Docker Ready ================"
cd /home/catkin_ws
exec "$@"
