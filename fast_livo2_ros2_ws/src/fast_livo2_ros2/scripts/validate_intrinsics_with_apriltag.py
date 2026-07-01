#!/usr/bin/python3
"""Validate camera intrinsics by AprilTag pose and corner reprojection."""

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


def _default_camera_yaml() -> Path:
    return Path(get_package_share_directory("fast_livo2_ros2")) / "config" / "camera_pinhole_rgb.yaml"


def _load_camera_yaml(path: Path) -> tuple[np.ndarray, np.ndarray, tuple[int, int]]:
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    params = data
    if "fast_livo2_mapping" in params:
        params = params["fast_livo2_mapping"]["ros__parameters"]
    elif "ros__parameters" in params:
        params = params["ros__parameters"]

    fx = float(params["cam_fx"])
    fy = float(params["cam_fy"])
    cx = float(params["cam_cx"])
    cy = float(params["cam_cy"])
    width = int(params["cam_width"])
    height = int(params["cam_height"])
    dist = np.array(
        [
            float(params.get("cam_d0", 0.0)),
            float(params.get("cam_d1", 0.0)),
            float(params.get("cam_d2", 0.0)),
            float(params.get("cam_d3", 0.0)),
            float(params.get("cam_d4", 0.0)),
        ],
        dtype=np.float64,
    )
    camera_matrix = np.array(
        [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    return camera_matrix, dist, (width, height)


FAMILY_KEYS = {
    "tag16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "tag25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "tag36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "tag36h11": cv2.aruco.DICT_APRILTAG_36h11,
}


def _make_dictionary(name: str):
    key = FAMILY_KEYS[name]
    return cv2.aruco.Dictionary_get(key)


def _rotation_degrees(rvec: np.ndarray) -> tuple[float, float, float]:
    rmat, _ = cv2.Rodrigues(rvec)
    sy = math.sqrt(rmat[0, 0] * rmat[0, 0] + rmat[1, 0] * rmat[1, 0])
    singular = sy < 1e-6
    if not singular:
        roll = math.atan2(rmat[2, 1], rmat[2, 2])
        pitch = math.atan2(-rmat[2, 0], sy)
        yaw = math.atan2(rmat[1, 0], rmat[0, 0])
    else:
        roll = math.atan2(-rmat[1, 2], rmat[1, 1])
        pitch = math.atan2(-rmat[2, 0], sy)
        yaw = 0.0
    return tuple(math.degrees(v) for v in (roll, pitch, yaw))


class AprilTagIntrinsicValidator(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("apriltag_intrinsic_validator")
        self.args = args
        self.bridge = CvBridge()
        self.camera_matrix, self.dist_coeffs, self.expected_size = _load_camera_yaml(args.camera_yaml)
        family_names = list(FAMILY_KEYS.keys()) if args.family == "auto" else [args.family]
        self.dictionaries = {name: _make_dictionary(name) for name in family_names}
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.parameters.adaptiveThreshWinSizeMin = 3
        self.parameters.adaptiveThreshWinSizeMax = 53
        self.parameters.adaptiveThreshWinSizeStep = 4
        self.parameters.minMarkerPerimeterRate = 0.015
        self.parameters.maxMarkerPerimeterRate = 4.0
        self.parameters.polygonalApproxAccuracyRate = 0.05
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.parameters.cornerRefinementWinSize = 5
        self.last_log_time = 0.0
        self.error_window: list[float] = []
        self.distance_window: list[float] = []
        self.target_id: Optional[int] = args.tag_id
        self.target_family: Optional[str] = None

        self.subscription = self.create_subscription(Image, args.topic, self.image_callback, 10)
        self.get_logger().info(
            f"Listening on {args.topic}; camera_yaml={args.camera_yaml}; "
            f"family={args.family}; tag_size={args.tag_size_m:.6f} m"
        )
        self.get_logger().info(
            "Move the tag from image center to four corners. Press q in the image window to quit."
        )

    def image_callback(self, msg: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        height, width = frame.shape[:2]
        if (width, height) != self.expected_size:
            self.get_logger().warn(
                f"Image size is {width}x{height}, but YAML is "
                f"{self.expected_size[0]}x{self.expected_size[1]}. Intrinsics are not valid for this stream.",
                throttle_duration_sec=2.0,
            )

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections, rejected_count = self._detect(gray)
        display = frame.copy()
        if detections:
            for family, corners, ids in detections:
                cv2.aruco.drawDetectedMarkers(display, corners, ids)
            selected = self._select_detection(detections)
            if selected is not None:
                tag_corners, tag_id, family = selected
                self._evaluate_detection(display, tag_corners, tag_id, family)
        else:
            cv2.putText(
                display,
                f"no AprilTag; rejected candidates={rejected_count}",
                (20, 36),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )

        if not self.args.no_gui:
            cv2.imshow("AprilTag intrinsic validation", display)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                rclpy.shutdown()

    def _detect(self, gray: np.ndarray) -> tuple[list[tuple[str, list[np.ndarray], np.ndarray]], int]:
        detections = []
        rejected_count = 0
        for family, dictionary in self.dictionaries.items():
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, dictionary, parameters=self.parameters)
            rejected_count += len(rejected)
            if ids is not None and len(ids) > 0:
                detections.append((family, corners, ids))
        return detections, rejected_count

    def _select_detection(
        self, detections: list[tuple[str, list[np.ndarray], np.ndarray]]
    ) -> Optional[tuple[np.ndarray, int, str]]:
        candidates = []
        for family, corners, ids in detections:
            for idx, tag_id in enumerate(ids.reshape(-1)):
                area = cv2.contourArea(corners[idx].reshape(4, 2).astype(np.float32))
                candidates.append((area, corners[idx], int(tag_id), family))
        if not candidates:
            return None

        if self.target_id is None:
            _, tag_corners, tag_id, family = max(candidates, key=lambda item: item[0])
            self.target_id = tag_id
            self.target_family = family
            self.get_logger().info(f"Using first visible tag family={family}, id={tag_id}")
            return tag_corners, tag_id, family

        matching = [
            item
            for item in candidates
            if item[2] == self.target_id and (self.target_family is None or item[3] == self.target_family)
        ]
        if matching:
            _, tag_corners, tag_id, family = max(matching, key=lambda item: item[0])
            return tag_corners, tag_id, family

        if self.args.tag_id is None:
            _, tag_corners, tag_id, family = max(candidates, key=lambda item: item[0])
            self.target_id = tag_id
            self.target_family = family
            self.get_logger().info(f"Switching to visible tag family={family}, id={tag_id}")
            return tag_corners, tag_id, family
        return None

    def _evaluate_detection(self, display: np.ndarray, corners: np.ndarray, tag_id: int, family: str) -> None:
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            [corners],
            float(self.args.tag_size_m),
            self.camera_matrix,
            self.dist_coeffs,
        )
        rvec = rvecs[0, 0, :].reshape(3, 1)
        tvec = tvecs[0, 0, :].reshape(3, 1)
        cv2.drawFrameAxes(
            display,
            self.camera_matrix,
            self.dist_coeffs,
            rvec,
            tvec,
            float(self.args.tag_size_m) * 0.5,
        )

        half = float(self.args.tag_size_m) * 0.5
        object_points = np.array(
            [[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]],
            dtype=np.float64,
        )
        projected, _ = cv2.projectPoints(object_points, rvec, tvec, self.camera_matrix, self.dist_coeffs)
        projected = projected.reshape(4, 2)
        observed = corners.reshape(4, 2)
        per_corner = np.linalg.norm(projected - observed, axis=1)
        reproj_error = float(np.mean(per_corner))

        self.error_window.append(reproj_error)
        self.distance_window.append(float(np.linalg.norm(tvec)))
        self.error_window = self.error_window[-60:]
        self.distance_window = self.distance_window[-60:]

        for pt in projected:
            cv2.circle(display, tuple(np.round(pt).astype(int)), 4, (255, 0, 255), -1)

        roll, pitch, yaw = _rotation_degrees(rvec)
        distance = float(np.linalg.norm(tvec))
        status = (
            f"{family} id={tag_id} dist={distance:.3f}m reproj={reproj_error:.2f}px "
            f"avg60={np.mean(self.error_window):.2f}px"
        )
        cv2.putText(display, status, (20, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (0, 255, 0), 2, cv2.LINE_AA)

        now = time.monotonic()
        if now - self.last_log_time >= self.args.log_period:
            self.last_log_time = now
            self.get_logger().info(
                f"family={family}, id={tag_id}, distance={distance:.4f} m, "
                f"t=[{tvec[0,0]:.4f}, {tvec[1,0]:.4f}, {tvec[2,0]:.4f}], "
                f"rpy_deg=[{roll:.2f}, {pitch:.2f}, {yaw:.2f}], "
                f"reprojection_mean={reproj_error:.3f} px, reprojection_max={np.max(per_corner):.3f} px, "
                f"avg60={np.mean(self.error_window):.3f} px, dist_std60={np.std(self.distance_window):.4f} m"
            )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/left_camera/image", help="ROS2 image topic.")
    parser.add_argument(
        "--camera-yaml",
        type=Path,
        default=_default_camera_yaml(),
        help="FAST-LIVO2 camera intrinsic YAML.",
    )
    parser.add_argument(
        "--tag-size-m",
        type=float,
        required=True,
        help="Measured AprilTag black outer square side length in meters.",
    )
    parser.add_argument("--tag-id", type=int, default=None, help="Expected tag id. Default: first visible tag.")
    parser.add_argument(
        "--family",
        choices=("auto", "tag16h5", "tag25h9", "tag36h10", "tag36h11"),
        default="auto",
        help="AprilTag family. Default scans common families automatically.",
    )
    parser.add_argument("--log-period", type=float, default=1.0, help="Console logging period in seconds.")
    parser.add_argument("--no-gui", action="store_true", help="Run without OpenCV image window.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = AprilTagIntrinsicValidator(args)
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
