#!/usr/bin/python3
"""Overlay LiDAR points on the camera image for quick extrinsic checks."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2


def _nested_params(data: dict) -> dict:
    if "fast_livo2_mapping" in data:
        return data["fast_livo2_mapping"]["ros__parameters"]
    if "ros__parameters" in data:
        return data["ros__parameters"]
    return data


def _nested_extrin(data: dict) -> dict:
    params = _nested_params(data)
    if "extrin_calib" in params:
        return params["extrin_calib"]
    if "extrin_calib" in data:
        return data["extrin_calib"]
    return params


def _load_camera_yaml(path: Path) -> tuple[np.ndarray, np.ndarray, tuple[int, int]]:
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    params = _nested_params(data)
    camera_matrix = np.array(
        [
            [float(params["cam_fx"]), 0.0, float(params["cam_cx"])],
            [0.0, float(params["cam_fy"]), float(params["cam_cy"])],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    dist_coeffs = np.array(
        [
            float(params.get("cam_d0", 0.0)),
            float(params.get("cam_d1", 0.0)),
            float(params.get("cam_d2", 0.0)),
            float(params.get("cam_d3", 0.0)),
            float(params.get("cam_d4", 0.0)),
        ],
        dtype=np.float64,
    )
    image_size = (
        int(params.get("cam_width", 0)),
        int(params.get("cam_height", 0)),
    )
    return camera_matrix, dist_coeffs, image_size


def _load_extrinsic_yaml(path: Path) -> tuple[np.ndarray, np.ndarray]:
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    extrin = _nested_extrin(data)
    rcl = np.asarray(extrin["Rcl"], dtype=np.float64).reshape(3, 3)
    pcl = np.asarray(extrin["Pcl"], dtype=np.float64).reshape(3)
    return rcl, pcl


def _cloud_to_xyz(msg: PointCloud2) -> np.ndarray:
    try:
        arr = point_cloud2.read_points_numpy(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True,
        )
        arr = np.asarray(arr)
    except Exception:
        pts = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        arr = np.asarray(pts if isinstance(pts, np.ndarray) else list(pts))

    if arr.size == 0:
        return np.empty((0, 3), dtype=np.float64)
    if arr.dtype.fields:
        return np.column_stack((arr["x"], arr["y"], arr["z"])).astype(np.float64)
    return arr.astype(np.float64, copy=False).reshape(-1, 3)


def _depth_color(depth: np.ndarray, min_depth: float, max_depth: float) -> np.ndarray:
    span = max(max_depth - min_depth, 1.0e-6)
    normalized = np.clip((depth - min_depth) / span, 0.0, 1.0)
    values = (255.0 * (1.0 - normalized)).astype(np.uint8)
    colors = cv2.applyColorMap(values.reshape(-1, 1), cv2.COLORMAP_TURBO).reshape(-1, 3)
    return colors


class ProjectionVisualizer(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("lidar_camera_projection_visualizer")
        self.args = args
        self.bridge = CvBridge()
        self.camera_matrix, self.dist_coeffs, self.calib_size = _load_camera_yaml(args.camera_yaml)
        self.rcl, self.pcl = _load_extrinsic_yaml(args.extrinsic_yaml)
        if args.ignore_distortion:
            self.dist_coeffs[:] = 0.0

        self.latest_cloud: Optional[PointCloud2] = None
        self.latest_image: Optional[Image] = None
        self.warned_scale = False

        qos = rclpy.qos.qos_profile_sensor_data
        self.create_subscription(Image, args.image_topic, self._image_cb, qos)
        self.create_subscription(PointCloud2, args.cloud_topic, self._cloud_cb, qos)
        self.publisher = self.create_publisher(Image, args.output_topic, 5)
        self.timer = self.create_timer(1.0 / args.publish_rate, self._timer_cb)

        self.get_logger().info(f"image_topic={args.image_topic}")
        self.get_logger().info(f"cloud_topic={args.cloud_topic}")
        self.get_logger().info(f"output_topic={args.output_topic}")
        self.get_logger().info(f"Rcl={self.rcl.reshape(-1).tolist()}")
        self.get_logger().info(f"Pcl={self.pcl.tolist()}")

    def _image_cb(self, msg: Image) -> None:
        self.latest_image = msg

    def _cloud_cb(self, msg: PointCloud2) -> None:
        self.latest_cloud = msg

    def _scaled_camera_matrix(self, width: int, height: int) -> np.ndarray:
        if self.calib_size[0] <= 0 or self.calib_size[1] <= 0:
            return self.camera_matrix.copy()
        if (width, height) == self.calib_size:
            return self.camera_matrix.copy()
        scale_x = width / float(self.calib_size[0])
        scale_y = height / float(self.calib_size[1])
        matrix = self.camera_matrix.copy()
        matrix[0, 0] *= scale_x
        matrix[0, 2] *= scale_x
        matrix[1, 1] *= scale_y
        matrix[1, 2] *= scale_y
        if not self.warned_scale:
            self.get_logger().warn(
                f"image size {width}x{height} differs from calibration "
                f"{self.calib_size[0]}x{self.calib_size[1]}; scaled intrinsics for visualization"
            )
            self.warned_scale = True
        return matrix

    def _timer_cb(self) -> None:
        if self.latest_image is None or self.latest_cloud is None:
            self.get_logger().info("waiting for image and PointCloud2...", throttle_duration_sec=3.0)
            return

        frame = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding="bgr8")
        height, width = frame.shape[:2]
        camera_matrix = self._scaled_camera_matrix(width, height)
        overlay = frame.copy()

        lidar_points = _cloud_to_xyz(self.latest_cloud)
        if lidar_points.size == 0:
            return

        finite = np.isfinite(lidar_points).all(axis=1)
        lidar_points = lidar_points[finite]
        ranges = np.linalg.norm(lidar_points, axis=1)
        mask = ranges > self.args.zero_point_epsilon_m
        mask &= ranges >= self.args.min_lidar_range
        mask &= ranges <= self.args.max_lidar_range
        lidar_points = lidar_points[mask]
        if lidar_points.shape[0] == 0:
            return

        if lidar_points.shape[0] > self.args.max_points:
            stride = int(math.ceil(lidar_points.shape[0] / self.args.max_points))
            lidar_points = lidar_points[::stride]

        camera_points = (self.rcl @ lidar_points.T).T + self.pcl
        depth = camera_points[:, 2]
        valid_depth = (depth > self.args.min_camera_depth) & (depth < self.args.max_camera_depth)
        camera_points = camera_points[valid_depth]
        depth = depth[valid_depth]
        if camera_points.shape[0] == 0:
            cv2.putText(
                overlay,
                "camera_front_points=0; check Rcl or LiDAR frame axes",
                (16, 32),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
            out_msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
            out_msg.header = self.latest_image.header
            self.publisher.publish(out_msg)
            if self.args.show_window:
                cv2.imshow("LiDAR -> camera projection", overlay)
                cv2.waitKey(1)
            return

        image_points, _ = cv2.projectPoints(
            camera_points,
            np.zeros(3, dtype=np.float64),
            np.zeros(3, dtype=np.float64),
            camera_matrix,
            self.dist_coeffs,
        )
        image_points = image_points.reshape(-1, 2)
        in_image = (
            (image_points[:, 0] >= 0.0) &
            (image_points[:, 0] < width) &
            (image_points[:, 1] >= 0.0) &
            (image_points[:, 1] < height)
        )
        image_points = image_points[in_image]
        depth = depth[in_image]
        if image_points.shape[0] == 0:
            cv2.putText(
                overlay,
                "projected=0; check Rcl/Pcl, frame axes, or range limits",
                (16, 32),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
            out_msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
            out_msg.header = self.latest_image.header
            self.publisher.publish(out_msg)
            if self.args.show_window:
                cv2.imshow("LiDAR -> camera projection", overlay)
                cv2.waitKey(1)
            return

        colors = _depth_color(depth, self.args.min_camera_depth, self.args.max_camera_depth)
        radius = int(max(1, self.args.point_radius))
        for point, color in zip(image_points.astype(np.int32), colors):
            cv2.circle(overlay, (int(point[0]), int(point[1])), radius, color.tolist(), thickness=-1)

        text = (
            f"projected={image_points.shape[0]} "
            f"cloud_stamp={self.latest_cloud.header.stamp.sec}.{self.latest_cloud.header.stamp.nanosec:09d}"
        )
        cv2.putText(
            overlay,
            text,
            (16, 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            overlay,
            "near=red/yellow  far=blue/purple",
            (16, 64),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        out_msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
        out_msg.header = self.latest_image.header
        self.publisher.publish(out_msg)

        if self.args.show_window:
            cv2.imshow("LiDAR -> camera projection", overlay)
            cv2.waitKey(1)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--image-topic", default="/left_camera/image")
    parser.add_argument("--cloud-topic", default="/livox/lidar")
    parser.add_argument("--output-topic", default="/lidar_camera_projection/image")
    parser.add_argument(
        "--camera-yaml",
        type=Path,
        default=Path("src/fast_livo2_ros2/config/camera_pinhole_rgb.yaml"),
    )
    parser.add_argument(
        "--extrinsic-yaml",
        type=Path,
        default=Path("calibration/manual_lidar_camera_extrinsic.yaml"),
    )
    parser.add_argument("--publish-rate", type=float, default=5.0)
    parser.add_argument("--max-points", type=int, default=15000)
    parser.add_argument("--point-radius", type=int, default=2)
    parser.add_argument("--zero-point-epsilon-m", type=float, default=0.03)
    parser.add_argument("--min-lidar-range", type=float, default=0.3)
    parser.add_argument("--max-lidar-range", type=float, default=8.0)
    parser.add_argument("--min-camera-depth", type=float, default=0.2)
    parser.add_argument("--max-camera-depth", type=float, default=8.0)
    parser.add_argument("--ignore-distortion", action="store_true")
    parser.add_argument("--show-window", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    rclpy.init()
    node = ProjectionVisualizer(args)
    try:
        rclpy.spin(node)
    finally:
        if args.show_window:
            cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
