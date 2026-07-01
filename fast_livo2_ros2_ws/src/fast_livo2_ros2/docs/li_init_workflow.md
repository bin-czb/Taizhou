# LI-Init Workflow

LI-Init is the HKU-MARS LiDAR-IMU initialization package. It estimates the
LiDAR-IMU temporal offset, extrinsic transform, gravity vector, and IMU bias.
It does not need a calibration board.

## Current Status

The current workspace is ROS2 Humble. Official LI-Init is a ROS1/catkin package
and depends on the ROS1 `livox_ros_driver`, so it cannot be launched directly
from this workspace.

What can be done now:

- record ROS2 LiDAR+IMU bags for later LI-Init use;
- verify IMU units, axes, and frequency;
- keep the recommended LiDAR-IMU extrinsic record.

What still needs an extra environment:

- ROS1 Noetic/Melodic with `catkin_make`, or the official Docker flow;
- `livox_ros_driver` message compatibility;
- ROS2 bag conversion or a live bridge from ROS2 topics to ROS1 topics.

## Record Data Now

Start Livox in `CustomMsg` mode and start the external IMU:

```bash
cd "/home/czb/Tai Zhou/fast_livo2_ros2_ws"
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch livox_ros_driver2 msg_MID360s_launch.py
```

```bash
cd "/home/czb/Tai Zhou/fast_livo2_ros2_ws"
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch fast_livo2_ros2 hwt601_485_imu.launch.py \
  port:=/dev/ttyUSB0 \
  baudrate:=115200 \
  address:=80 \
  rate_hz:=150.0 \
  accel_range_g:=4.0
```

Record:

```bash
cd "/home/czb/Tai Zhou/fast_livo2_ros2_ws"
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run fast_livo2_ros2 record_lidar_imu_init_bag.sh
```

Motion sequence:

1. Keep the rig completely still for at least 5 seconds.
2. Rotate around roll, pitch, and yaw.
3. Move forward/back, left/right, and up/down.
4. Record 60-120 seconds in an area with enough LiDAR structure.

## Official LI-Init Run Shape

In a ROS1 workspace, the official flow is:

```bash
cd ~/catkin_ws/src
git clone https://github.com/hku-mars/LiDAR_IMU_Init.git
cd ..
catkin_make -j
source devel/setup.bash
roslaunch lidar_imu_init xxx.launch
```

Before running, edit the chosen YAML:

- `lid_topic`: LiDAR topic;
- `imu_topic`: IMU topic;
- `mean_acc_norm`: use about `9.805` for this external IMU;
- `cut_frame_num` and `orig_odom_freq`: keep their product around `50` for
  Livox;
- `online_refine_time`: commonly 15-30 seconds.

The official result is written to:

```text
catkin_ws/src/LiDAR_IMU_Init/result/Initialization_result.txt
```

The useful outputs for downstream FAST-LIVO2-style systems are:

- LiDAR-IMU extrinsic rotation;
- LiDAR-IMU extrinsic translation;
- LiDAR-to-IMU time offset.

## Practical Recommendation

For the current setup, official LI-Init has already been run on:

```text
/home/czb/fast_livo2_validation_bags/lidar_imu_init_150hz_20260622_145550
```

Saved result:

```text
calibration/li_init_lidar_imu_result_20260622_150hz.yaml
calibration/recommended_lidar_imu_extrinsic_20260622.yaml
```

Recommended use:

- use LI-Init rotation;
- use mechanical translation `[0.0, 0.0, 0.08]` first;
- treat LI-Init time lag `-0.053331 s` as a candidate until repeated recordings
  confirm it.
