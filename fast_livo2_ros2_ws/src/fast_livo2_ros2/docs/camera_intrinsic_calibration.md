# Camera Intrinsic Calibration With Checkerboard

This workflow calibrates the USB RGB camera from `/left_camera/image`.

Printed target:

- Checkerboard inner corners: `9 x 6`
- Measured square size: `0.020 m`
- Keep the checkerboard flat on a hard board.

Available calibration results:

- `640x480`: lower CPU load, smoother preview, less image detail.
- `1280x720`: selected as the default for LiDAR-camera extrinsic calibration and FAST-LIVO2 because it gives better image detail for projection and target localization.

Do not mix resolutions. Intrinsics calibrated at one resolution are not valid for another resolution unless explicitly scaled and revalidated.

## 1. Start Camera

The external USB camera was detected as `/dev/video2`.

Terminal 1:

```bash
conda deactivate  # optional, only if the shell shows (base)
cd "/home/czb/Tai Zhou/fast_livo2_ros2_ws"
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run fast_livo2_ros2 start_usb_camera.sh 2 1280 720 10.0 /left_camera/image
```

If the camera cannot open, check whether another process is using it:

```bash
fuser -v /dev/video2 /dev/video3
```

## 2. Visualize Camera

Terminal 2:

```bash
conda deactivate  # optional, only if the shell shows (base)
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
unset PYTHONHOME
source /opt/ros/humble/setup.bash
source "/home/czb/Tai Zhou/fast_livo2_ros2_ws/install/setup.bash"

ROS_LOG_DIR=/tmp/rqt_image_view_logs \
ros2 run rqt_image_view rqt_image_view /left_camera/image
```

Check that the checkerboard is sharp, fully visible, and not overexposed.

## 3. Optional Bag Recording

Recording a bag is useful for traceability:

```bash
conda deactivate  # optional, only if the shell shows (base)
cd "/home/czb/Tai Zhou/fast_livo2_ros2_ws"
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run fast_livo2_ros2 record_camera_intrinsic_bag.sh /left_camera/image
```

## 4. Live Calibration

Terminal 3:

```bash
conda deactivate  # optional, only if the shell shows (base)
cd "/home/czb/Tai Zhou/fast_livo2_ros2_ws"
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
unset PYTHONHOME
source /opt/ros/humble/setup.bash
source install/setup.bash

ROS_LOG_DIR=/tmp/checkerboard_calib_logs \
ros2 run fast_livo2_ros2 calibrate_checkerboard_from_topic.py \
  --topic /left_camera/image \
  --inner-cols 9 \
  --inner-rows 6 \
  --square-size-m 0.020 \
  --min-frames 25 \
  --max-frames 60
```

Move the checkerboard through the camera view:

- center, left, right, top, bottom, and all four corners,
- near and far,
- tilted left/right and up/down,
- keep the board still for a short moment when the script sees corners.

Press `q` in the OpenCV window when enough frames are collected, or wait for `max-frames`.

## 5. Outputs

The script writes a timestamped folder under:

```text
calibration/camera_intrinsic/
```

Files:

- `camera_intrinsic_opencv.yaml`: full OpenCV result.
- `camera_pinhole_rgb.yaml`: FAST-LIVO2-ready config fragment.
- `summary.txt`: readable summary.
- `accepted_frames/`: raw accepted frames.
- `corner_debug/`: accepted frames with drawn corners.

Use `camera_pinhole_rgb.yaml` to update:

```text
src/fast_livo2_ros2/config/camera_pinhole_rgb.yaml
```

## Acceptance

Good calibration usually has:

```text
mean_reprojection_error_px < 0.5 - 0.8
```

If the error is larger:

- collect more views near image corners,
- avoid blurry frames,
- ensure square size is really `0.020 m`,
- keep the checkerboard flat,
- do not change camera resolution after calibration.

## Current Saved Results

See:

```text
calibration/camera_intrinsic/README.md
```

The default runtime config is:

```text
src/fast_livo2_ros2/config/camera_pinhole_rgb.yaml
```

It currently points to the 1280x720 calibration.
