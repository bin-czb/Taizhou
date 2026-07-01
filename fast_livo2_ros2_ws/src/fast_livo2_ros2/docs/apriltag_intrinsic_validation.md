# AprilTag Intrinsic Validation

This procedure validates the selected FAST-LIVO2 camera intrinsics with a printed
AprilTag. It does not replace the checkerboard calibration; it checks whether the
YAML intrinsics match the live camera stream.

## Required Measurements

- Use a square AprilTag, preferably `tag36h11`.
- Measure the black outer square side length with a ruler or caliper.
- Use meters in the command. For example, 10 cm is `0.10`.
- Do not use the full paper size or white margin size as the tag size.

## Start The Camera

```bash
cd "/home/czb/Tai Zhou/fast_livo2_ros2_ws"
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run fast_livo2_ros2 start_usb_camera.sh 2 1280 720 10.0 /left_camera/image
```

In a second terminal:

```bash
cd "/home/czb/Tai Zhou/fast_livo2_ros2_ws"
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic hz /left_camera/image
```

The live image resolution must match `config/camera_pinhole_rgb.yaml`.

## Run Validation

Replace `0.100` with the measured black outer square side length.

```bash
cd "/home/czb/Tai Zhou/fast_livo2_ros2_ws"
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run fast_livo2_ros2 validate_intrinsics_with_apriltag.py \
  --topic /left_camera/image \
  --camera-yaml src/fast_livo2_ros2/config/camera_pinhole_rgb.yaml \
  --family auto \
  --tag-size-m 0.100
```

Optional: lock to a known tag id.

```bash
ros2 run fast_livo2_ros2 validate_intrinsics_with_apriltag.py \
  --topic /left_camera/image \
  --camera-yaml src/fast_livo2_ros2/config/camera_pinhole_rgb.yaml \
  --family auto \
  --tag-id 0 \
  --tag-size-m 0.100
```

If the window still shows `no AprilTag`, check the printed board first:

- the tag itself must be square;
- the black outer border side length is the value passed to `--tag-size-m`;
- keep a white margin around the tag;
- keep the board flat and well lit;
- start around 0.5-1.0 m from the camera;
- make sure the printed image is an AprilTag, not a checkerboard or an ArUco marker.

## Motion Pattern

Hold the board still for several seconds at each pose:

1. Image center, board nearly parallel to the camera.
2. Left, right, top, and bottom of the image.
3. Four image corners.
4. Mild tilt up/down and left/right.
5. Distances around 0.5 m, 1.0 m, and 1.5 m if the tag remains clearly visible.

## Acceptance

- The script must not warn about image size mismatch.
- Static board distance should not jump strongly.
- `reprojection_mean` should usually stay below about `0.5 px` near the center.
- Near image edges, about `1.0 px` is still usable for a USB camera.
- If edge error is much larger than center error, distortion or resolution is likely wrong.
- If measured distance is consistently wrong by a large ratio, check `--tag-size-m` first.

The magenta dots are reprojected tag corners. They should overlap the detected
tag corners. Large visual separation means the live stream and YAML intrinsics do
not agree.
