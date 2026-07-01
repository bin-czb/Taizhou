# Camera Intrinsic Calibration Index

Two valid checkerboard calibrations are available. Do not mix resolutions.

## 640x480

Source folder:

```text
checkerboard_20260618_185724/
```

Result:

```text
image_size: 640 x 480
checkerboard_inner_corners: 9 x 6
square_size_m: 0.020000
fx fy cx cy: 470.396088 466.769371 321.688507 241.362717
distortion k1 k2 p1 p2 k3: -0.039706826 0.063478139 0.002692837 0.003021267 0.000000000
rms_error: 0.376309
mean_reprojection_error_px: 0.039800
```

FAST-LIVO2 config copy:

```text
src/fast_livo2_ros2/config/camera_pinhole_rgb_640x480.yaml
```

Use only with:

```bash
ros2 run fast_livo2_ros2 start_usb_camera.sh 2 640 480 15.0 /left_camera/image
```

## 1280x720

Source folder:

```text
checkerboard_20260618_190320/
```

Result:

```text
image_size: 1280 x 720
checkerboard_inner_corners: 9 x 6
square_size_m: 0.020000
fx fy cx cy: 715.799008 708.732198 647.586881 342.577746
distortion k1 k2 p1 p2 k3: -0.054322996 0.090084176 -0.004535688 0.004554663 0.000000000
rms_error: 0.720477
mean_reprojection_error_px: 0.080207
```

FAST-LIVO2 config copy:

```text
src/fast_livo2_ros2/config/camera_pinhole_rgb_1280x720.yaml
```

Use with:

```bash
ros2 run fast_livo2_ros2 start_usb_camera.sh 2 1280 720 10.0 /left_camera/image
```

## Default Choice

The default `src/fast_livo2_ros2/config/camera_pinhole_rgb.yaml` currently points to the 1280x720 calibration.

Use 1280x720 for LiDAR-camera extrinsic calibration and FAST-LIVO2 if the computer can sustain it. It gives more image detail for point projection and AprilTag/board edge localization.

Use 640x480 only if runtime performance is more important than projection precision, and then collect LiDAR-camera extrinsic data at 640x480 as well.
