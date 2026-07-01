# System Integration Contract

This document is the wiring contract for recording raw validation data that will later be consumed by the real localization/navigation system. Keep it updated when topic names, frames, calibration files, or ownership boundaries change.

## Ownership

`fast_livo2_ros2` now owns only calibration helpers, bring-up checks, and rosbag recording helpers. It does not own localization, mission planning, global navigation, map serving, or robot control.

Recommended subsystem split:

| Subsystem | Owns | Provides to bag/downstream system | Notes |
| --- | --- | --- | --- |
| Camera driver | RGB camera settings and images | `/left_camera/image` | Resolution must match calibration. |
| Livox driver | Mid360 packet parsing and point timestamps | `/livox/lidar` | Prefer `livox_ros_driver2/msg/CustomMsg`. |
| IMU driver/filter | Raw external IMU stream | `/imu/data` | Keep frame and covariance policy documented. |
| Bag recorder | Raw topic capture | rosbag directory | Store calibration YAML next to the bag. |
| Downstream localization | Pose/map estimation | consumes bag or live raw topics | Implemented outside this package. |

## Required Recorded Inputs

Default input contract:

| Topic | Type | Notes |
| --- | --- | --- |
| `/left_camera/image` | `sensor_msgs/msg/Image` | Runtime resolution must match `camera_pinhole_rgb.yaml`. |
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | Preferred for Mid360 because per-point `offset_time` is preserved. |
| `/imu/data` | `sensor_msgs/msg/Imu` | External IMU. Orientation covariance policy should be documented by the IMU owner. |

Alternative LiDAR input may be recorded as `sensor_msgs/msg/PointCloud2`, but it is not preferred because Livox per-point timing may be lost.

## Frames

Recommended raw frame contract:

```text
base_link
  -> livox_frame
  -> left_camera
  -> imu_link
```

If TF is not available during recording, the bag is still useful as long as the calibration YAML records the sensor-to-sensor transforms.

## Calibration Artifacts

Store calibration by robot and date. Recommended layout:

```text
calibration/
  robot_001/
    2026-06-16/
      camera_pinhole_rgb.yaml
      mid360_rgb_apriltag.yaml
      apriltag_lidar_scenes.yaml
      recording_topics.yaml
      README.md
```

Each calibration README should record:

- camera model, lens, resolution, and exposure/focus mode,
- AprilTag/Aprilgrid board dimensions,
- Mid360 mount pose and LiDAR serial number,
- external IMU model, mount pose, and timestamp source,
- whether time sync is hardware, software-estimated, or unset,
- bag names used to produce the calibration.

## Recording Composition

Recommended bring-up order:

1. Camera driver.
2. Livox Mid360 driver.
3. External IMU driver.
4. Optional TF/static transform publisher if available.
5. Topic checker.
6. Rosbag recorder.

Use `check_record_topics.py` before recording. It catches missing or mismatched raw input topics early.

The Livox driver is now built inside this workspace as `src/livox_ros_driver2`. See `livox_mid360_bringup.md` before connecting hardware.

## Multi-System Notes

- Keep raw sensor topics available in rosbag recordings. Do not record only derived outputs.
- For formal tests, record the exact config YAML files next to each rosbag.
