# Validation Rosbag Recording

This workspace is now used for calibration and raw sensor data capture. It does not run FAST-LIVO2 localization.

## Required Raw Topics

| Topic | Type | Purpose |
| --- | --- | --- |
| `/left_camera/image` | `sensor_msgs/msg/Image` | RGB image stream using calibrated resolution. |
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | Mid360 point cloud with per-point offset time. |
| `/livox/imu` | `sensor_msgs/msg/Imu` | Livox built-in IMU, recorded for comparison/diagnostics. |
| `/imu/data` | `sensor_msgs/msg/Imu` | External IMU stream, target 150 Hz. |

Optional topics recorded when present:

```text
/tf
/tf_static
```

## Before Recording

Start each device in separate terminals:

```bash
cd "/home/czb/Tai Zhou/fast_livo2_ros2_ws"
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Camera, using the calibrated 1280x720 mode:

```bash
ros2 run fast_livo2_ros2 start_usb_camera.sh 2 1280 720 30.0 /left_camera/image
```

Livox Mid360:

```bash
ros2 launch livox_ros_driver2 msg_MID360s_launch.py
```

HWT601 RS485 IMU:

```bash
ros2 launch fast_livo2_ros2 hwt601_485_imu.launch.py port:=/dev/ttyUSB0 baudrate:=115200 address:=80 rate_hz:=150.0 accel_range_g:=4.0
```

Check topic presence:

```bash
ros2 run fast_livo2_ros2 check_record_topics.py
```

Check topic frequency:

```bash
ros2 topic hz /left_camera/image
ros2 topic hz /livox/lidar
ros2 topic hz /imu/data
```

Optional RViz camera/TF check:

```bash
ros2 launch fast_livo2_ros2 rviz_sensor_recording.launch.py
```

## Record

```bash
ros2 run fast_livo2_ros2 record_fast_livo_site_bag.sh
```

By default, bags are written under:

```text
~/fast_livo2_validation_bags/validation_YYYYMMDD_HHMMSS
```

To choose a directory and session name:

```bash
ros2 run fast_livo2_ros2 record_fast_livo_site_bag.sh ~/fast_livo2_validation_bags site_nav_01
```

The script also writes:

```text
~/fast_livo2_validation_bags/site_nav_01_calibration_snapshot
```

Keep this snapshot with the bag. It records the camera intrinsics, LiDAR-camera
extrinsic, LiDAR-IMU extrinsic, and recording-topic contract used for that run.

## Verify A Bag

```bash
ros2 bag info ~/fast_livo2_validation_bags/scene_01_static
```

The bag must contain at least:

```text
/left_camera/image
/livox/lidar
/livox/imu
/imu/data
```

For downstream localization tests, keep a copy of the calibration YAML files next to each bag:

```text
camera_pinhole_rgb.yaml
mid360_rgb_apriltag.yaml
recording_topics.yaml
```

## Record LiDAR-IMU Initialization Data

For later LI-Init use, record only LiDAR and IMU with a dedicated session name:

```bash
ros2 run fast_livo2_ros2 record_lidar_imu_init_bag.sh
```

Keep the rig still for at least 5 seconds, then excite roll, pitch, yaw,
forward/back, left/right, and up/down motion for 60-120 seconds.
