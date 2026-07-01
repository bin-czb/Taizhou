# fast_livo2_ros2

ROS2 Humble calibration and validation-data acquisition utilities for Mid360 + RGB camera + external IMU.

This package no longer runs FAST-LIVO2 localization. Its current job is to make the raw inputs reliable, record bags, and preserve calibration files for the downstream localization system.

Current task boundary:

- Calibrate camera intrinsics and LiDAR-camera extrinsics.
- Bring up USB RGB camera, Livox Mid360, and HWT601 RS485 IMU.
- Verify topic names, types, and rates before recording.
- Record raw rosbag data for later FAST-LIVO2 or navigation-system validation.

Required bag topics:

| Topic | Type |
| --- | --- |
| `/left_camera/image` | `sensor_msgs/msg/Image` |
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` |
| `/livox/imu` | `sensor_msgs/msg/Imu` |
| `/imu/data` | `sensor_msgs/msg/Imu` |

Current selected calibration:

- Camera intrinsics: `config/camera_pinhole_rgb.yaml`, 1280x720 checkerboard result.
- LiDAR-camera: `config/mid360_rgb_apriltag.yaml`, manual first-pass `Rcl/Pcl`.
- LiDAR-IMU: `config/mid360_rgb_apriltag.yaml`, LI-Init rotation plus mechanical 0.08 m translation.
- Time offset: left at zero for raw recording; LI-Init `-0.053331 s` is a candidate only.

Useful notes:

- `docs/camera_intrinsic_calibration.md`: checkerboard camera intrinsics.
- `docs/apriltag_intrinsic_validation.md`: AprilTag-based intrinsic validation.
- `docs/lidar_camera_extrinsic_calibration.md`: plane-based LiDAR-camera extrinsic calibration.
- `docs/livox_mid360_bringup.md`: Livox Mid-360 network and driver bring-up.
- `docs/hwt601_485_imu_bringup.md`: HWT601-AGV-485 / RS485 IMU probing and `/imu/data`.
- `docs/imu_calibration.md`: IMU static checks, axis checks, and LiDAR-IMU calibration path.
- `docs/li_init_workflow.md`: LI-Init usage notes and ROS2 bag recording workflow.
- `docs/data_recording.md`: validation rosbag recording workflow.
- `docs/system_integration.md`: raw-topic and calibration artifact contract.
- `launch/rviz_sensor_recording.launch.py`: RViz2 view for camera image and TF checks.

Quick recording flow:

```bash
cd "/home/czb/Tai Zhou/fast_livo2_ros2_ws"
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run fast_livo2_ros2 check_record_topics.py
ros2 run fast_livo2_ros2 record_fast_livo_site_bag.sh \
  "$HOME/fast_livo2_validation_bags" \
  "site_nav_$(date +%Y%m%d_%H%M%S)"
```

The script records `/left_camera/image`, `/livox/lidar`, `/livox/imu`, `/imu/data`,
`/tf`, and `/tf_static`, then keeps a calibration snapshot next to the bag.
