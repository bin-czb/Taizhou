#!/usr/bin/python3
"""Calibrate a pinhole RGB camera from a ROS2 image topic and checkerboard."""

from __future__ import annotations

import argparse
import signal
import sys
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


def _timestamp_dir(root: Path) -> Path:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return root / f"checkerboard_{stamp}"


class CheckerboardCalibrator(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("checkerboard_intrinsic_calibrator")
        self.args = args
        self.bridge = CvBridge()
        self.pattern_size = (args.inner_cols, args.inner_rows)
        self.object_template = self._make_object_points()
        self.object_points: list[np.ndarray] = []
        self.image_points: list[np.ndarray] = []
        self.image_size: Optional[tuple[int, int]] = None
        self.last_sample_time = 0.0
        self.last_corners: Optional[np.ndarray] = None
        self.frame_count = 0
        self.accepted_count = 0
        self.done = False

        self.output_dir = args.output_dir or _timestamp_dir(args.output_root)
        self.frames_dir = self.output_dir / "accepted_frames"
        self.corners_dir = self.output_dir / "corner_debug"
        self.frames_dir.mkdir(parents=True, exist_ok=True)
        self.corners_dir.mkdir(parents=True, exist_ok=True)

        self.subscription = self.create_subscription(
            Image,
            args.topic,
            self.image_callback,
            10,
        )
        self.get_logger().info(
            f"Listening on {args.topic}; pattern={args.inner_cols}x{args.inner_rows}, "
            f"square_size={args.square_size_m:.6f} m, output={self.output_dir}"
        )

    def _make_object_points(self) -> np.ndarray:
        objp = np.zeros((self.args.inner_cols * self.args.inner_rows, 3), np.float32)
        objp[:, :2] = (
            np.mgrid[0 : self.args.inner_cols, 0 : self.args.inner_rows]
            .T.reshape(-1, 2)
            .astype(np.float32)
        )
        objp *= float(self.args.square_size_m)
        return objp

    def image_callback(self, msg: Image) -> None:
        if self.done:
            return
        self.frame_count += 1
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.image_size = (gray.shape[1], gray.shape[0])

        found, corners = self._find_corners(gray)
        display = frame.copy()
        if found:
            cv2.drawChessboardCorners(display, self.pattern_size, corners, found)
            if self._should_accept(corners):
                self._accept_sample(frame, display, corners)
        if not self.args.no_gui:
            self._show(display, found)

        if self.accepted_count >= self.args.max_frames:
            self.done = True

    def _find_corners(self, gray: np.ndarray) -> tuple[bool, Optional[np.ndarray]]:
        if hasattr(cv2, "findChessboardCornersSB"):
            found, corners = cv2.findChessboardCornersSB(
                gray,
                self.pattern_size,
                flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY,
            )
            if found:
                return True, corners.astype(np.float32)

        flags = (
            cv2.CALIB_CB_ADAPTIVE_THRESH
            | cv2.CALIB_CB_NORMALIZE_IMAGE
            | cv2.CALIB_CB_FAST_CHECK
        )
        found, corners = cv2.findChessboardCorners(gray, self.pattern_size, flags)
        if not found:
            return False, None
        criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            30,
            0.001,
        )
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        return True, corners

    def _should_accept(self, corners: np.ndarray) -> bool:
        now = time.monotonic()
        if now - self.last_sample_time < self.args.sample_period:
            return False
        if self.last_corners is not None:
            delta = np.mean(np.linalg.norm(corners.reshape(-1, 2) - self.last_corners, axis=1))
            if delta < self.args.min_corner_motion_px:
                return False
        return True

    def _accept_sample(self, frame: np.ndarray, debug: np.ndarray, corners: np.ndarray) -> None:
        self.accepted_count += 1
        self.object_points.append(self.object_template.copy())
        self.image_points.append(corners.reshape(-1, 1, 2).astype(np.float32))
        self.last_corners = corners.reshape(-1, 2).copy()
        self.last_sample_time = time.monotonic()

        name = f"frame_{self.accepted_count:03d}.png"
        cv2.imwrite(str(self.frames_dir / name), frame)
        cv2.imwrite(str(self.corners_dir / name), debug)
        self.get_logger().info(
            f"Accepted frame {self.accepted_count}/{self.args.max_frames} "
            f"(seen {self.frame_count} images)"
        )

    def _show(self, display: np.ndarray, found: bool) -> None:
        status = (
            f"found={found} accepted={self.accepted_count}/{self.args.max_frames} "
            "move board around; press q to finish"
        )
        cv2.putText(
            display,
            status,
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0) if found else (0, 0, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.imshow("checkerboard intrinsic calibration", display)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            self.done = True

    def calibrate_and_save(self) -> bool:
        if self.image_size is None:
            self.get_logger().error("No images received; cannot calibrate.")
            return False
        if len(self.image_points) < self.args.min_frames:
            self.get_logger().error(
                f"Only {len(self.image_points)} valid frames; need at least {self.args.min_frames}."
            )
            return False

        flags = 0
        if self.args.fix_k3:
            flags |= cv2.CALIB_FIX_K3
        rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            self.object_points,
            self.image_points,
            self.image_size,
            None,
            None,
            flags=flags,
        )
        mean_error, per_view_errors = self._reprojection_errors(
            camera_matrix, dist_coeffs, rvecs, tvecs
        )
        self._write_outputs(camera_matrix, dist_coeffs, rms, mean_error, per_view_errors)
        return True

    def _reprojection_errors(
        self,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        rvecs: list[np.ndarray],
        tvecs: list[np.ndarray],
    ) -> tuple[float, list[float]]:
        errors = []
        for objp, imgp, rvec, tvec in zip(self.object_points, self.image_points, rvecs, tvecs):
            projected, _ = cv2.projectPoints(objp, rvec, tvec, camera_matrix, dist_coeffs)
            err = cv2.norm(imgp, projected, cv2.NORM_L2) / len(projected)
            errors.append(float(err))
        return float(np.mean(errors)), errors

    def _write_outputs(
        self,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        rms: float,
        mean_error: float,
        per_view_errors: list[float],
    ) -> None:
        dist = dist_coeffs.reshape(-1).astype(float)
        while len(dist) < 5:
            dist = np.append(dist, 0.0)
        width, height = self.image_size
        fx, fy = float(camera_matrix[0, 0]), float(camera_matrix[1, 1])
        cx, cy = float(camera_matrix[0, 2]), float(camera_matrix[1, 2])
        k1, k2, p1, p2 = map(float, dist[:4])

        opencv_yaml = {
            "image_width": int(width),
            "image_height": int(height),
            "checkerboard": {
                "inner_cols": self.args.inner_cols,
                "inner_rows": self.args.inner_rows,
                "square_size_m": float(self.args.square_size_m),
            },
            "camera_matrix": {
                "rows": 3,
                "cols": 3,
                "data": [float(v) for v in camera_matrix.reshape(-1)],
            },
            "distortion_model": "plumb_bob",
            "distortion_coefficients": {
                "rows": 1,
                "cols": 5,
                "data": [float(v) for v in dist[:5]],
            },
            "rms_error": float(rms),
            "mean_reprojection_error_px": float(mean_error),
            "per_view_reprojection_error_px": per_view_errors,
            "accepted_frames": len(self.image_points),
        }

        fast_livo_yaml = {
            "fast_livo2_mapping": {
                "ros__parameters": {
                    "cam_model": "Pinhole",
                    "cam_width": int(width),
                    "cam_height": int(height),
                    "scale": 1.0,
                    "cam_fx": fx,
                    "cam_fy": fy,
                    "cam_cx": cx,
                    "cam_cy": cy,
                    "cam_d0": k1,
                    "cam_d1": k2,
                    "cam_d2": p1,
                    "cam_d3": p2,
                }
            }
        }

        (self.output_dir / "camera_intrinsic_opencv.yaml").write_text(
            yaml.safe_dump(opencv_yaml, sort_keys=False), encoding="utf-8"
        )
        (self.output_dir / "camera_pinhole_rgb.yaml").write_text(
            yaml.safe_dump(fast_livo_yaml, sort_keys=False), encoding="utf-8"
        )

        summary = (
            f"frames: {len(self.image_points)}\n"
            f"image_size: {width} x {height}\n"
            f"checkerboard_inner_corners: {self.args.inner_cols} x {self.args.inner_rows}\n"
            f"square_size_m: {self.args.square_size_m:.6f}\n"
            f"fx fy cx cy: {fx:.6f} {fy:.6f} {cx:.6f} {cy:.6f}\n"
            f"distortion k1 k2 p1 p2 k3: {dist[0]:.9f} {dist[1]:.9f} "
            f"{dist[2]:.9f} {dist[3]:.9f} {dist[4]:.9f}\n"
            f"rms_error: {rms:.6f}\n"
            f"mean_reprojection_error_px: {mean_error:.6f}\n"
            f"fast_livo_yaml: {self.output_dir / 'camera_pinhole_rgb.yaml'}\n"
        )
        (self.output_dir / "summary.txt").write_text(summary, encoding="utf-8")
        self.get_logger().info("\\n" + summary)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/left_camera/image")
    parser.add_argument("--inner-cols", type=int, default=9)
    parser.add_argument("--inner-rows", type=int, default=6)
    parser.add_argument("--square-size-m", type=float, default=0.020)
    parser.add_argument("--output-root", type=Path, default=Path("calibration/camera_intrinsic"))
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--min-frames", type=int, default=25)
    parser.add_argument("--max-frames", type=int, default=60)
    parser.add_argument("--sample-period", type=float, default=0.7)
    parser.add_argument("--min-corner-motion-px", type=float, default=18.0)
    parser.add_argument("--fix-k3", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--no-gui", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = CheckerboardCalibrator(args)

    def stop(_signum, _frame):
        node.done = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        ok = node.calibrate_and_save()
        return 0 if ok else 2
    finally:
        if not args.no_gui:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
