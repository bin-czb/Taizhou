# LiDAR-Camera Extrinsic Calibration

This package provides a first-pass plane-based LiDAR-camera calibration helper.
It estimates:

```text
p_camera = Rcl * p_lidar + Pcl
```

The method uses multiple static poses of the same planar board:

- camera side: AprilTag pose gives the board plane in the camera frame;
- LiDAR side: PointCloud2 is segmented into multiple plane candidates;
- candidates are scored with board size, plane distance, and inlier count;
- 3 or more different board angles are optimized with nonlinear least squares to solve `Rcl/Pcl`.

## Target

Use a rigid flat board. A cardboard box can work for first-pass calibration, but
a flat hard board is better.

The LiDAR does not see the AprilTag pattern. It only sees the board plane. The
tag only needs to be detected by the camera.

Current board:

```text
family: tagStandard41h12
id: 0
box face: 0.341 m x 0.233 m
tag black outer square: 0.100 m
left/right/top/bottom offsets: 0.1225 / 0.1220 / 0.0675 / 0.0650 m
```

The board file is:

```text
config/box_tagstandard41h12_id0.yaml
```

Required measurement for any new board:

```text
tag black outer square side length, in meters
```

Board width/height and tag-to-edge distances are still worth recording in the
calibration README, especially if you later switch to a corner/rectangle based
calibration.

## Start Sensors

Use the PointCloud2 Livox launch while calibrating:

```bash
cd "/home/czb/Tai Zhou/fast_livo2_ros2_ws"
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch livox_ros_driver2 rviz_MID360s_launch.py
```

Start the camera in another terminal:

```bash
cd "/home/czb/Tai Zhou/fast_livo2_ros2_ws"
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run fast_livo2_ros2 start_usb_camera.sh 2 1280 720 10.0 /left_camera/image
```

Check:

```bash
ros2 topic hz /left_camera/image
ros2 topic hz /livox/lidar
ros2 topic info /livox/lidar
```

For this script, `/livox/lidar` must be:

```text
sensor_msgs/msg/PointCloud2
```

## Run Calibration

Start with a crop that keeps the board but removes as much floor/wall clutter as
possible.

```bash
cd "/home/czb/Tai Zhou/fast_livo2_ros2_ws"
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run fast_livo2_ros2 calibrate_lidar_camera_planes.py \
  --board-yaml src/fast_livo2_ros2/config/box_tagstandard41h12_id0.yaml \
  --captures 8 \
  --crop-x-min 0.3 --crop-x-max 1.4 \
  --crop-y-min 0.2 --crop-y-max 1.2 \
  --crop-z-min 0.2 --crop-z-max 0.9 \
  --output calibration/lidar_camera_planes.yaml
```

`tagStandard41h12` is not supported by OpenCV's built-in ArUco dictionaries on
this machine. Install one native AprilTag Python binding before running:

```bash
sudo apt install python3-apriltag
```

or:

```bash
pip install pupil-apriltags
```

For each pose:

1. Move the board to a new static pose.
2. Keep it still.
3. Press Enter in the calibration terminal.

Use 8-15 scenes. Vary:

```text
left/right
up/down
near/far
tilted left/right
tilted up/down
```

## Crop The LiDAR Board Points

The script prints candidate planes for every capture:

```text
ok[0] d=0.850 centroid=[...] extent=0.34x0.23 inliers=...
reject[0] d=0.001 centroid=[...] extent=...
```

For your current board, a good candidate should roughly have:

```text
d:       about 0.5-1.5 m, close to camera d
extent:  about 0.34 x 0.23 m, not several meters
centroid: not near [0, 0, 0]
```

If the script fits the wall, floor, or nearby clutter instead of the board,
restrict the LiDAR crop further. Example:

```bash
ros2 run fast_livo2_ros2 calibrate_lidar_camera_planes.py \
  --board-yaml src/fast_livo2_ros2/config/box_tagstandard41h12_id0.yaml \
  --captures 8 \
  --crop-x-min 0.4 --crop-x-max 1.2 \
  --crop-y-min 0.3 --crop-y-max 1.0 \
  --crop-z-min 0.25 --crop-z-max 0.8 \
  --output calibration/lidar_camera_planes.yaml
```

The output contains:

```yaml
extrin_calib:
  Rcl: [...]
  Pcl: [...]
```

Copy those values into:

```text
config/mid360_rgb_apriltag.yaml
```

## Acceptance

After at least 3 scenes, the script prints residuals. For a first-pass setup:

```text
worst normal residual: ideally < 2 deg
worst plane distance residual: ideally < 0.05 m
```

If residuals are worse:

- make the board flatter and more rigid;
- improve lighting so AprilTag detection is stable;
- crop LiDAR points tighter around the board;
- collect more tilted board poses;
- avoid poses where the LiDAR sees very few board points.

## Important Note

Plane-only calibration is good for first-pass extrinsics and data collection
validation. For final high-accuracy LIVO work, validate by projecting LiDAR
points into the image and checking that board edges align.
