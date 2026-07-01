#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BAG_NAME="${1:-lidar_imu_init_20260621_165848_ros1.bag}"

docker run --rm \
  --net=host \
  --ipc=host \
  --shm-size=1gb \
  --volume="${WORKSPACE_ROOT}/third_party:/home/catkin_ws/src:rw" \
  --volume="${WORKSPACE_ROOT}/converted_bags_ros1:/bags:rw" \
  li_init:noetic-local \
  bash -lc "
    set -eo pipefail
    catkin_make -j2
    source devel/setup.bash
    rm -f /home/catkin_ws/src/LiDAR_IMU_Init/result/Initialization_result.txt
    roslaunch lidar_imu_init livox_mid360.launch rviz:=false &
    launch_pid=\$!
    sleep 8
    rosbag play --clock --quiet /bags/${BAG_NAME}
    sleep 5
    kill \$launch_pid >/dev/null 2>&1 || true
    wait \$launch_pid >/dev/null 2>&1 || true
    if [ -f /home/catkin_ws/src/LiDAR_IMU_Init/result/Initialization_result.txt ]; then
      echo '================ LI-Init result ================'
      cat /home/catkin_ws/src/LiDAR_IMU_Init/result/Initialization_result.txt
    else
      echo 'LI-Init result file was not generated.' >&2
      exit 2
    fi
  "
