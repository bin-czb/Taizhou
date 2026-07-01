#!/usr/bin/python3
"""Estimate LiDAR->camera extrinsics from AprilTag camera planes and LiDAR planes.

This tool is intended for static scenes. Start the camera and the Livox
PointCloud2 visualization launch, place the tagged planar board at a new pose,
wait until both streams are stable, and press Enter to capture one scene.
"""

from __future__ import annotations

import argparse
import itertools
import math
import select
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2


OPENCV_FAMILY_KEYS = {
    "tag16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "tag25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "tag36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "tag36h11": cv2.aruco.DICT_APRILTAG_36h11,
}
NATIVE_FAMILIES = {"tagStandard41h12", "tagStandard52h13"}
SUPPORTED_FAMILIES = sorted(set(OPENCV_FAMILY_KEYS) | NATIVE_FAMILIES)


@dataclass
class Plane:
    normal: np.ndarray
    distance: float
    centroid: np.ndarray
    inliers: int
    total: int
    extent_major: float = 0.0
    extent_minor: float = 0.0
    score: float = 0.0


@dataclass
class PointGroup:
    name: str
    points: np.ndarray
    count: int
    extent_xyz: np.ndarray
    centroid: np.ndarray
    accepted_for_ransac: bool
    reason: str = ""


@dataclass
class Scene:
    name: str
    image_stamp: float
    cloud_stamp: float
    camera_plane: Plane
    lidar_plane: Plane


class BoundsOnlyRequested(RuntimeError):
    pass


def _load_board_yaml(path: Optional[Path]) -> dict:
    if path is None:
        return {}
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    return data or {}


def _board_value(board: dict, key: str, fallback=None):
    value = board
    for part in key.split("."):
        if not isinstance(value, dict) or part not in value:
            return fallback
        value = value[part]
    return value


def _stamp_seconds(msg) -> float:
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1.0e-9


def _load_camera_yaml(path: Path) -> tuple[np.ndarray, np.ndarray]:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    params = data
    if "fast_livo2_mapping" in params:
        params = params["fast_livo2_mapping"]["ros__parameters"]
    elif "ros__parameters" in params:
        params = params["ros__parameters"]

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
    return camera_matrix, dist_coeffs


def _make_dictionary(name: str):
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(OPENCV_FAMILY_KEYS[name])
    return cv2.aruco.Dictionary_get(OPENCV_FAMILY_KEYS[name])


def _make_detector_parameters():
    if hasattr(cv2.aruco, "DetectorParameters"):
        params = cv2.aruco.DetectorParameters()
    else:
        params = cv2.aruco.DetectorParameters_create()
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 53
    params.adaptiveThreshWinSizeStep = 4
    params.minMarkerPerimeterRate = 0.015
    params.maxMarkerPerimeterRate = 4.0
    params.polygonalApproxAccuracyRate = 0.05
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return params


def _detect_markers(gray: np.ndarray, dictionary, params):
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, params)
        return detector.detectMarkers(gray)
    return cv2.aruco.detectMarkers(gray, dictionary, parameters=params)


def _order_corners_tl_tr_br_bl(corners: np.ndarray) -> np.ndarray:
    points = corners.reshape(4, 2).astype(np.float64)
    sums = points.sum(axis=1)
    diffs = points[:, 0] - points[:, 1]
    ordered = np.array(
        [
            points[int(np.argmin(sums))],
            points[int(np.argmax(diffs))],
            points[int(np.argmax(sums))],
            points[int(np.argmin(diffs))],
        ],
        dtype=np.float64,
    )
    return ordered


def _detect_with_native_apriltag(gray: np.ndarray, family: str, tag_id: Optional[int]) -> np.ndarray:
    try:
        from pupil_apriltags import Detector as PupilDetector

        detector = PupilDetector(
            families=family,
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
        )
        detections = detector.detect(gray)
        matches = [
            det for det in detections
            if tag_id is None or int(det.tag_id) == int(tag_id)
        ]
        if not matches:
            raise RuntimeError(f"AprilTag family={family} id={tag_id} not detected")
        selected = max(matches, key=lambda det: cv2.contourArea(np.asarray(det.corners, dtype=np.float32)))
        return _order_corners_tl_tr_br_bl(np.asarray(selected.corners, dtype=np.float64))
    except ModuleNotFoundError:
        pass

    try:
        import apriltag

        if hasattr(apriltag, "Detector"):
            detector = apriltag.Detector(apriltag.DetectorOptions(families=family))
            detections = detector.detect(gray)
            matches = [
                det for det in detections
                if tag_id is None or int(det.tag_id) == int(tag_id)
            ]
            if not matches:
                raise RuntimeError(f"AprilTag family={family} id={tag_id} not detected")
            selected = max(
                matches,
                key=lambda det: cv2.contourArea(np.asarray(det.corners, dtype=np.float32)),
            )
            return _order_corners_tl_tr_br_bl(np.asarray(selected.corners, dtype=np.float64))

        if hasattr(apriltag, "apriltag"):
            detector = apriltag.apriltag(family)
            detections = detector.detect(gray)
            matches = [
                det for det in detections
                if tag_id is None or int(det.get("id", -1)) == int(tag_id)
            ]
            if not matches:
                raise RuntimeError(f"AprilTag family={family} id={tag_id} not detected")
            selected = max(
                matches,
                key=lambda det: cv2.contourArea(np.asarray(det["lb-rb-rt-lt"], dtype=np.float32)),
            )
            lb_rb_rt_lt = np.asarray(selected["lb-rb-rt-lt"], dtype=np.float64)
            lb, rb, rt, lt = lb_rb_rt_lt
            return np.array([lt, rt, rb, lb], dtype=np.float64)

        raise RuntimeError("imported apriltag module has no supported detector API")
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            f"{family} is not supported by OpenCV on this machine. Install one detector binding first: "
            "sudo apt install python3-apriltag, or pip install pupil-apriltags"
        ) from exc


def _detect_tag_corners(gray: np.ndarray, family: str, tag_id: Optional[int]) -> np.ndarray:
    if family in OPENCV_FAMILY_KEYS:
        dictionary = _make_dictionary(family)
        params = _make_detector_parameters()
        corners, ids, _ = _detect_markers(gray, dictionary, params)
        if ids is None or len(ids) == 0:
            raise RuntimeError(f"no AprilTag detected for family={family}")

        ids_flat = ids.reshape(-1)
        if tag_id is None:
            index = int(np.argmax([cv2.contourArea(c.reshape(4, 2).astype(np.float32)) for c in corners]))
        else:
            matches = np.where(ids_flat == tag_id)[0]
            if len(matches) == 0:
                raise RuntimeError(f"AprilTag id={tag_id} not detected")
            index = int(matches[0])
        return _order_corners_tl_tr_br_bl(corners[index])

    if family in NATIVE_FAMILIES:
        return _detect_with_native_apriltag(gray, family, tag_id)

    raise RuntimeError(f"unsupported AprilTag family={family}")


def _camera_plane_from_apriltag(
    image_msg: Image,
    bridge: CvBridge,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    family: str,
    tag_id: Optional[int],
    tag_size_m: float,
) -> Plane:
    frame = bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    image_points = _detect_tag_corners(gray, family, tag_id)
    half = tag_size_m * 0.5
    object_points = np.array(
        [
            [-half, -half, 0.0],
            [half, -half, 0.0],
            [half, half, 0.0],
            [-half, half, 0.0],
        ],
        dtype=np.float64,
    )
    ok, rvec, tvec = cv2.solvePnP(
        object_points,
        image_points,
        camera_matrix,
        dist_coeffs,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok:
        raise RuntimeError("solvePnP failed for AprilTag corners")

    rotation, _ = cv2.Rodrigues(rvec)
    normal = rotation[:, 2].reshape(3)
    point = tvec.reshape(3)
    # Use a consistent plane normal pointing from the board toward the sensor origin.
    if float(normal.dot(point)) > 0.0:
        normal *= -1.0
    normal = normal / np.linalg.norm(normal)
    distance = -float(normal.dot(point))
    return Plane(normal=normal, distance=distance, centroid=point, inliers=4, total=4)


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


def _crop_points(points: np.ndarray, args: argparse.Namespace) -> np.ndarray:
    mask = np.isfinite(points).all(axis=1)
    mask &= np.linalg.norm(points, axis=1) > args.zero_point_epsilon_m
    mask &= points[:, 0] >= args.crop_x_min
    mask &= points[:, 0] <= args.crop_x_max
    mask &= points[:, 1] >= args.crop_y_min
    mask &= points[:, 1] <= args.crop_y_max
    mask &= points[:, 2] >= args.crop_z_min
    mask &= points[:, 2] <= args.crop_z_max
    return points[mask]


def _load_background_points(path: Path, args: argparse.Namespace) -> tuple[np.ndarray, cKDTree]:
    points = np.asarray(np.load(path), dtype=np.float64).reshape(-1, 3)
    mask = np.isfinite(points).all(axis=1)
    mask &= np.linalg.norm(points, axis=1) > args.zero_point_epsilon_m
    points = points[mask]
    if points.shape[0] == 0:
        raise RuntimeError(f"background cloud {path} has no usable xyz points")
    if points.shape[0] > args.background_max_points:
        rng = np.random.default_rng(args.ransac_seed)
        indices = rng.choice(points.shape[0], args.background_max_points, replace=False)
        points = points[indices]
    return points, cKDTree(points)


def _subtract_background(points: np.ndarray, args: argparse.Namespace) -> tuple[np.ndarray, int]:
    tree = getattr(args, "background_tree", None)
    if tree is None or points.size == 0:
        return points, 0
    distances, _ = tree.query(points, k=1)
    keep = distances > args.background_distance_threshold_m
    removed = int(points.shape[0] - np.count_nonzero(keep))
    return points[keep], removed


def _make_point_group(name: str, points: np.ndarray, args: argparse.Namespace) -> PointGroup:
    if points.size == 0:
        extent = np.zeros(3, dtype=np.float64)
        centroid = np.zeros(3, dtype=np.float64)
    else:
        mins = points.min(axis=0)
        maxs = points.max(axis=0)
        extent = maxs - mins
        centroid = points.mean(axis=0)

    max_extent = float(np.max(extent)) if extent.size else 0.0
    min_required_extent = float(args.foreground_cluster_min_extent_m)
    max_allowed_extent = float(args.foreground_cluster_max_extent_m)
    accepted = True
    reasons = []
    if points.shape[0] < args.min_lidar_points:
        accepted = False
        reasons.append(f"points_{points.shape[0]}<{args.min_lidar_points}")
    if max_extent < min_required_extent:
        accepted = False
        reasons.append(f"max_extent_{max_extent:.3f}<{min_required_extent:.3f}")
    if max_extent > max_allowed_extent:
        accepted = False
        reasons.append(f"max_extent_{max_extent:.3f}>{max_allowed_extent:.3f}")

    return PointGroup(
        name=name,
        points=points,
        count=int(points.shape[0]),
        extent_xyz=extent,
        centroid=centroid,
        accepted_for_ransac=accepted,
        reason=",".join(reasons) if reasons else "pass",
    )


def _foreground_point_groups(points: np.ndarray, args: argparse.Namespace) -> list[PointGroup]:
    if not args.cluster_foreground or points.shape[0] < args.min_lidar_points:
        return [_make_point_group("all_foreground", points, args)]

    voxel = max(float(args.foreground_cluster_voxel_m), 0.02)
    voxel_to_indices: dict[tuple[int, int, int], list[int]] = {}
    voxel_coords = np.floor(points / voxel).astype(np.int64)
    for idx, coord in enumerate(voxel_coords):
        key = (int(coord[0]), int(coord[1]), int(coord[2]))
        voxel_to_indices.setdefault(key, []).append(idx)

    unvisited = set(voxel_to_indices)
    neighbor_offsets = [
        offset for offset in itertools.product((-1, 0, 1), repeat=3)
        if offset != (0, 0, 0)
    ]
    groups: list[PointGroup] = []
    cluster_idx = 0
    while unvisited:
        start = unvisited.pop()
        queue = [start]
        component_keys = [start]
        while queue:
            current = queue.pop()
            for offset in neighbor_offsets:
                neighbor = (
                    current[0] + offset[0],
                    current[1] + offset[1],
                    current[2] + offset[2],
                )
                if neighbor not in unvisited:
                    continue
                unvisited.remove(neighbor)
                queue.append(neighbor)
                component_keys.append(neighbor)

        indices: list[int] = []
        for key in component_keys:
            indices.extend(voxel_to_indices[key])
        cluster_idx += 1
        cluster_points = points[np.asarray(indices, dtype=np.int64)]
        groups.append(_make_point_group(f"cluster_{cluster_idx:03d}", cluster_points, args))

    groups.sort(
        key=lambda group: (
            not group.accepted_for_ransac,
            abs(float(np.max(group.extent_xyz)) - max(_board_dimensions(args))),
            -group.count,
        )
    )
    return groups


def _print_foreground_groups(groups: list[PointGroup], args: argparse.Namespace) -> None:
    if not args.print_foreground_clusters:
        return
    print(f"Foreground clusters: total={len(groups)}")
    for idx, group in enumerate(groups[: args.print_foreground_cluster_limit]):
        status = "accept" if group.accepted_for_ransac else "reject"
        print(
            f"  {status}[{idx}] {group.name} count={group.count} "
            f"centroid=[{group.centroid[0]:.3f},{group.centroid[1]:.3f},{group.centroid[2]:.3f}] "
            f"extent_xyz=[{group.extent_xyz[0]:.3f},{group.extent_xyz[1]:.3f},"
            f"{group.extent_xyz[2]:.3f}] reason={group.reason}"
        )


def _format_cloud_bounds(points: np.ndarray) -> str:
    finite = points[np.isfinite(points).all(axis=1)]
    if finite.size == 0:
        return "cloud has no finite xyz points"
    percentiles = np.percentile(finite, [1, 5, 50, 95, 99], axis=0)
    mins = finite.min(axis=0)
    maxs = finite.max(axis=0)
    lines = [
        f"finite_points={finite.shape[0]}",
        f"min=[{mins[0]:.3f}, {mins[1]:.3f}, {mins[2]:.3f}]",
        f"max=[{maxs[0]:.3f}, {maxs[1]:.3f}, {maxs[2]:.3f}]",
    ]
    for label, row in zip(["p01", "p05", "p50", "p95", "p99"], percentiles):
        lines.append(f"{label}=[{row[0]:.3f}, {row[1]:.3f}, {row[2]:.3f}]")
    return "; ".join(lines)


def _format_roi_diagnostics(points: np.ndarray, camera_plane: Plane, args: argparse.Namespace) -> str:
    finite = points[np.isfinite(points).all(axis=1)]
    if finite.size == 0:
        return "ROI diagnostics: cloud has no finite xyz points"

    nonzero = finite[np.linalg.norm(finite, axis=1) > args.zero_point_epsilon_m]
    lines = [
        "ROI diagnostics:",
        (
            f"  camera board plane distance={camera_plane.distance:.3f} m; "
            f"tag center in camera=[{camera_plane.centroid[0]:.3f},"
            f"{camera_plane.centroid[1]:.3f},{camera_plane.centroid[2]:.3f}]"
        ),
        f"  nonzero lidar points={nonzero.shape[0]}/{finite.shape[0]}",
    ]
    if nonzero.size == 0:
        return "\n".join(lines)

    ranges = np.linalg.norm(nonzero, axis=1)
    center = camera_plane.distance
    for window in [0.25, 0.50, 0.80, 1.20]:
        mask = np.abs(ranges - center) <= window
        subset = nonzero[mask]
        if subset.shape[0] == 0:
            lines.append(f"  range around camera d +/- {window:.2f} m: count=0")
            continue
        lines.append(
            f"  range around camera d +/- {window:.2f} m: "
            f"count={subset.shape[0]}; {_format_cloud_bounds(subset)}"
        )

    roi_min = max(0.05, center - args.roi_range_window_m)
    roi_max = center + args.roi_range_window_m
    near = nonzero[(ranges >= roi_min) & (ranges <= roi_max)]
    if near.shape[0] == 0:
        lines.append(
            f"  occupied voxels in range [{roi_min:.2f},{roi_max:.2f}] m: count=0"
        )
        return "\n".join(lines)

    voxel = max(args.roi_voxel_size_m, 0.02)
    voxel_indices = np.floor(near / voxel).astype(np.int64)
    unique, inverse, counts = np.unique(voxel_indices, axis=0, return_inverse=True, return_counts=True)
    order = np.argsort(counts)[::-1][: args.roi_voxel_limit]
    lines.append(
        f"  densest {len(order)} voxels in range [{roi_min:.2f},{roi_max:.2f}] m "
        f"(voxel={voxel:.2f} m):"
    )
    for rank, unique_index in enumerate(order):
        pts = near[inverse == unique_index]
        mins = pts.min(axis=0)
        maxs = pts.max(axis=0)
        center_pt = 0.5 * (mins + maxs)
        lines.append(
            f"    voxel[{rank}] count={counts[unique_index]} "
            f"center=[{center_pt[0]:.3f},{center_pt[1]:.3f},{center_pt[2]:.3f}] "
            f"x=[{mins[0]:.3f},{maxs[0]:.3f}] "
            f"y=[{mins[1]:.3f},{maxs[1]:.3f}] "
            f"z=[{mins[2]:.3f},{maxs[2]:.3f}]"
        )
    return "\n".join(lines)


def _fit_plane_svd(points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    centroid = points.mean(axis=0)
    _, _, vt = np.linalg.svd(points - centroid, full_matrices=False)
    normal = vt[-1]
    normal = normal / np.linalg.norm(normal)
    if float(normal.dot(centroid)) > 0.0:
        normal *= -1.0
    return normal, centroid


def _plane_extents(points: np.ndarray, centroid: np.ndarray, normal: np.ndarray) -> tuple[float, float]:
    centered = points - centroid
    _, _, vt = np.linalg.svd(centered, full_matrices=False)
    axis_0 = vt[0]
    axis_1 = np.cross(normal, axis_0)
    axis_1_norm = np.linalg.norm(axis_1)
    if axis_1_norm < 1.0e-9:
        return 0.0, 0.0
    axis_1 /= axis_1_norm
    u = centered @ axis_0
    v = centered @ axis_1
    extents = sorted([float(u.max() - u.min()), float(v.max() - v.min())], reverse=True)
    return extents[0], extents[1]


def _make_plane_from_inliers(inlier_points: np.ndarray, total_count: int, args: argparse.Namespace) -> Plane:
    normal, centroid = _fit_plane_svd(inlier_points)
    distances = np.abs((inlier_points - centroid) @ normal)
    refined = inlier_points[distances < args.plane_threshold_m]
    if refined.shape[0] >= args.min_lidar_points:
        normal, centroid = _fit_plane_svd(refined)
        inlier_points = refined

    extent_major, extent_minor = _plane_extents(inlier_points, centroid, normal)
    distance = -float(normal.dot(centroid))
    return Plane(
        normal=normal,
        distance=distance,
        centroid=centroid,
        inliers=int(inlier_points.shape[0]),
        total=int(total_count),
        extent_major=extent_major,
        extent_minor=extent_minor,
    )


def _board_dimensions(args: argparse.Namespace) -> tuple[float, float]:
    width = float(_board_value(args.board, "box_face.width_m", 0.341))
    height = float(_board_value(args.board, "box_face.height_m", 0.233))
    return max(width, height), min(width, height)


def _score_plane_candidate(plane: Plane, camera_plane: Plane, args: argparse.Namespace) -> float:
    board_major, board_minor = _board_dimensions(args)
    extent_error = min(
        abs(plane.extent_major - board_major) / max(board_major, 1.0e-6) +
        abs(plane.extent_minor - board_minor) / max(board_minor, 1.0e-6),
        abs(plane.extent_major - board_minor) / max(board_minor, 1.0e-6) +
        abs(plane.extent_minor - board_major) / max(board_major, 1.0e-6),
    )
    distance_error = abs(plane.distance - camera_plane.distance)
    inlier_bonus = min(plane.inliers / max(args.min_lidar_points, 1), 8.0)
    return (
        args.candidate_extent_weight * extent_error +
        args.candidate_distance_weight * distance_error -
        args.candidate_inlier_weight * inlier_bonus
    )


def _plane_gate_status(plane: Plane, camera_plane: Plane, args: argparse.Namespace) -> tuple[bool, str]:
    board_major, board_minor = _board_dimensions(args)
    max_extent = args.max_board_extent_factor * board_major
    min_extent = args.min_board_extent_factor * board_minor
    distance_diff = abs(plane.distance - camera_plane.distance)
    failures = []
    if not (args.min_lidar_plane_distance_m <= plane.distance <= args.max_lidar_plane_distance_m):
        failures.append(
            f"lidar_d_not_in_[{args.min_lidar_plane_distance_m:.2f},{args.max_lidar_plane_distance_m:.2f}]"
        )
    if not (
        plane.extent_major <= max_extent and
        plane.extent_minor <= max_extent and
        plane.extent_major >= min_extent and
        plane.extent_minor >= min_extent
    ):
        failures.append(
            f"extent_not_in_[{min_extent:.2f},{max_extent:.2f}]"
        )
    if distance_diff > args.max_camera_lidar_plane_distance_diff_m:
        failures.append(
            f"camera_lidar_d_diff_{distance_diff:.3f}>"
            f"{args.max_camera_lidar_plane_distance_diff_m:.3f}"
        )
    return not failures, ",".join(failures) if failures else "pass"


def _extract_lidar_plane_candidates(
    points: np.ndarray,
    camera_plane: Plane,
    args: argparse.Namespace,
) -> list[Plane]:
    cropped = _crop_points(points, args)
    crop_count = cropped.shape[0]
    cropped, background_removed = _subtract_background(cropped, args)
    if background_removed and args.print_plane_candidates:
        print(
            f"Background subtraction: kept {cropped.shape[0]}/{crop_count} cropped points "
            f"(threshold={args.background_distance_threshold_m:.3f} m)"
        )
    if cropped.shape[0] < args.min_lidar_points:
        raise RuntimeError(
            f"not enough LiDAR points after crop: {cropped.shape[0]} < {args.min_lidar_points}. "
            f"raw_crop={crop_count}, background_removed={background_removed}. "
            f"crop x=[{args.crop_x_min:.3f},{args.crop_x_max:.3f}], "
            f"y=[{args.crop_y_min:.3f},{args.crop_y_max:.3f}], "
            f"z=[{args.crop_z_min:.3f},{args.crop_z_max:.3f}]. "
            f"Full cloud bounds: {_format_cloud_bounds(points)}\n"
            f"{_format_roi_diagnostics(points, camera_plane, args)}"
        )

    rng = np.random.default_rng(args.ransac_seed)
    groups = _foreground_point_groups(cropped, args)
    _print_foreground_groups(groups, args)
    search_groups = [group for group in groups if group.accepted_for_ransac]
    accepted: list[Plane] = []
    rejected: list[Plane] = []

    for group in search_groups[: args.max_foreground_clusters_for_ransac]:
        remaining = group.points.copy()
        planes_for_group = 1 if args.cluster_foreground else args.max_plane_candidates
        for _ in range(planes_for_group):
            if remaining.shape[0] < args.min_lidar_points:
                break

            if remaining.shape[0] > args.max_ransac_points:
                sample_idx = rng.choice(remaining.shape[0], args.max_ransac_points, replace=False)
                ransac_points = remaining[sample_idx]
            else:
                ransac_points = remaining

            best_inliers = None
            best_count = 0
            best_normal = None
            best_sample_point = None
            for _ in range(args.ransac_iterations):
                sample = ransac_points[rng.choice(ransac_points.shape[0], 3, replace=False)]
                v1 = sample[1] - sample[0]
                v2 = sample[2] - sample[0]
                normal = np.cross(v1, v2)
                norm = np.linalg.norm(normal)
                if norm < 1.0e-9:
                    continue
                normal /= norm
                distances = np.abs((remaining - sample[0]) @ normal)
                inliers = distances < args.plane_threshold_m
                count = int(np.count_nonzero(inliers))
                if count > best_count:
                    best_count = count
                    best_inliers = inliers
                    best_normal = normal
                    best_sample_point = sample[0]

            if best_inliers is None or best_count < args.min_lidar_points:
                break

            plane = _make_plane_from_inliers(remaining[best_inliers], group.count, args)
            plane.score = _score_plane_candidate(plane, camera_plane, args)

            passed, _ = _plane_gate_status(plane, camera_plane, args)
            if passed:
                accepted.append(plane)
            else:
                rejected.append(plane)

            remove_distances = np.abs((remaining - best_sample_point) @ best_normal)
            remaining = remaining[remove_distances > args.candidate_remove_threshold_m]

    candidates = sorted(accepted, key=lambda item: item.score)
    rejected = sorted(rejected, key=lambda item: item.score)
    if args.print_plane_candidates:
        print(f"LiDAR plane candidates (camera d={camera_plane.distance:.3f} m):")
        for idx, plane in enumerate(candidates[: args.print_plane_candidate_limit]):
            _, gate_status = _plane_gate_status(plane, camera_plane, args)
            distance_diff = abs(plane.distance - camera_plane.distance)
            print(
                f"  ok[{idx}] d={plane.distance:.3f} centroid="
                f"[{plane.centroid[0]:.3f},{plane.centroid[1]:.3f},{plane.centroid[2]:.3f}] "
                f"extent={plane.extent_major:.3f}x{plane.extent_minor:.3f} "
                f"inliers={plane.inliers}/{plane.total} diff={distance_diff:.3f} "
                f"score={plane.score:.3f} gate={gate_status}"
            )
        for idx, plane in enumerate(rejected[: args.print_plane_candidate_limit]):
            _, gate_status = _plane_gate_status(plane, camera_plane, args)
            distance_diff = abs(plane.distance - camera_plane.distance)
            print(
                f"  reject[{idx}] d={plane.distance:.3f} centroid="
                f"[{plane.centroid[0]:.3f},{plane.centroid[1]:.3f},{plane.centroid[2]:.3f}] "
                f"extent={plane.extent_major:.3f}x{plane.extent_minor:.3f} "
                f"inliers={plane.inliers}/{plane.total} diff={distance_diff:.3f} "
                f"score={plane.score:.3f} gate={gate_status}"
            )

    return candidates


def _fit_lidar_plane(points: np.ndarray, camera_plane: Plane, args: argparse.Namespace) -> Plane:
    candidates = _extract_lidar_plane_candidates(points, camera_plane, args)
    if not candidates:
        raise RuntimeError(
            "no LiDAR plane candidate passed board-size/distance gates. "
            "Tighten --crop-* around the calibration board, or relax "
            "--min-board-extent-factor/--max-board-extent-factor."
        )
    return candidates[0]


def _initial_solve_lidar_to_camera(scenes: list[Scene]) -> tuple[np.ndarray, np.ndarray]:
    lidar_normals = np.array([scene.lidar_plane.normal for scene in scenes])
    camera_normals = np.array([scene.camera_plane.normal for scene in scenes])

    h = np.zeros((3, 3), dtype=np.float64)
    for n_camera, n_lidar in zip(camera_normals, lidar_normals):
        h += np.outer(n_camera, n_lidar)
    u, _, vt = np.linalg.svd(h)
    rcl = u @ vt
    if np.linalg.det(rcl) < 0.0:
        u[:, -1] *= -1.0
        rcl = u @ vt

    a = camera_normals
    b = np.array(
        [scene.lidar_plane.distance - scene.camera_plane.distance for scene in scenes],
        dtype=np.float64,
    )
    pcl, *_ = np.linalg.lstsq(a, b, rcond=None)
    return rcl, pcl


def _solve_lidar_to_camera(
    scenes: list[Scene],
    args: argparse.Namespace,
) -> tuple[np.ndarray, np.ndarray, list[dict[str, float]]]:
    rcl_init, pcl_init = _initial_solve_lidar_to_camera(scenes)

    def residual_function(x: np.ndarray) -> np.ndarray:
        rotation = Rotation.from_rotvec(x[:3]).as_matrix()
        translation = x[3:6]
        residual_parts = []
        for scene in scenes:
            predicted_normal = rotation @ scene.lidar_plane.normal
            if predicted_normal.dot(scene.camera_plane.normal) < 0.0:
                predicted_normal *= -1.0
            residual_parts.extend(args.normal_residual_weight * (predicted_normal - scene.camera_plane.normal))
            distance_residual = scene.camera_plane.normal.dot(translation) - (
                scene.lidar_plane.distance - scene.camera_plane.distance
            )
            residual_parts.append(args.distance_residual_weight * distance_residual)
            centroid_residual = scene.camera_plane.normal.dot(
                rotation @ scene.lidar_plane.centroid + translation
            ) + scene.camera_plane.distance
            residual_parts.append(args.centroid_residual_weight * centroid_residual)
        return np.asarray(residual_parts, dtype=np.float64)

    x0 = np.zeros(6, dtype=np.float64)
    x0[:3] = Rotation.from_matrix(rcl_init).as_rotvec()
    x0[3:6] = pcl_init
    result = least_squares(
        residual_function,
        x0,
        loss="soft_l1",
        f_scale=0.05,
        max_nfev=200,
    )
    rcl = Rotation.from_rotvec(result.x[:3]).as_matrix()
    pcl = result.x[3:6]

    residuals = []
    for scene in scenes:
        predicted_normal = rcl @ scene.lidar_plane.normal
        if predicted_normal.dot(scene.camera_plane.normal) < 0.0:
            predicted_normal *= -1.0
        normal_error = scene.camera_plane.normal - predicted_normal
        normal_angle = math.degrees(
            math.acos(
                max(
                    -1.0,
                    min(1.0, float(scene.camera_plane.normal.dot(predicted_normal))),
                )
            )
        )
        distance_error = float(scene.camera_plane.normal.dot(pcl) - (
            scene.lidar_plane.distance - scene.camera_plane.distance
        ))
        centroid_error = float(scene.camera_plane.normal.dot(rcl @ scene.lidar_plane.centroid + pcl) + scene.camera_plane.distance)
        residuals.append(
            {
                "scene": scene.name,
                "normal_error_norm": float(np.linalg.norm(normal_error)),
                "normal_angle_deg": float(normal_angle),
                "distance_error_m": distance_error,
                "centroid_plane_error_m": centroid_error,
            }
        )
    return rcl, pcl, residuals


def _plane_to_yaml(plane: Plane) -> dict:
    return {
        "normal": [float(v) for v in plane.normal],
        "distance": float(plane.distance),
        "centroid": [float(v) for v in plane.centroid],
        "inliers": int(plane.inliers),
        "total": int(plane.total),
        "extent_major": float(plane.extent_major),
        "extent_minor": float(plane.extent_minor),
        "score": float(plane.score),
    }


def _write_output(
    path: Path,
    board: dict,
    scenes: list[Scene],
    rcl: Optional[np.ndarray],
    pcl: Optional[np.ndarray],
    residuals,
):
    output = {
        "method": "multi_plane_apriltag_lidar",
        "transform_convention": "LiDAR -> Camera: p_camera = Rcl * p_lidar + Pcl",
        "board": board,
        "scenes": [
            {
                "name": scene.name,
                "image_stamp": scene.image_stamp,
                "cloud_stamp": scene.cloud_stamp,
                "camera_plane": _plane_to_yaml(scene.camera_plane),
                "lidar_plane": _plane_to_yaml(scene.lidar_plane),
            }
            for scene in scenes
        ],
    }
    if rcl is not None and pcl is not None:
        output["extrin_calib"] = {
            "Rcl": [float(v) for v in rcl.reshape(-1)],
            "Pcl": [float(v) for v in pcl],
        }
        output["residuals"] = residuals
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(output, sort_keys=False), encoding="utf-8")


class CaptureNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("lidar_camera_plane_calibrator")
        self.args = args
        self.board = args.board
        self.bridge = CvBridge()
        self.camera_matrix, self.dist_coeffs = _load_camera_yaml(args.camera_yaml)
        self.latest_image: Optional[Image] = None
        self.latest_cloud: Optional[PointCloud2] = None
        self.cloud_buffer: list[PointCloud2] = []
        qos = rclpy.qos.qos_profile_sensor_data
        self.create_subscription(Image, args.image_topic, self._image_cb, qos)
        self.create_subscription(PointCloud2, args.cloud_topic, self._cloud_cb, qos)

    def _image_cb(self, msg: Image) -> None:
        self.latest_image = msg

    def _cloud_cb(self, msg: PointCloud2) -> None:
        self.latest_cloud = msg
        self.cloud_buffer.append(msg)
        latest_stamp = _stamp_seconds(msg)
        if self.args.cloud_accumulate_seconds > 0.0:
            self.cloud_buffer = [
                cloud for cloud in self.cloud_buffer
                if latest_stamp - _stamp_seconds(cloud) <= self.args.cloud_accumulate_seconds
            ]
        else:
            self.cloud_buffer = [msg]

    def _current_cloud_points(self) -> np.ndarray:
        if self.latest_cloud is None:
            raise RuntimeError(f"no PointCloud2 received on {self.args.cloud_topic}")
        if self.args.cloud_accumulate_seconds <= 0.0 or len(self.cloud_buffer) <= 1:
            return _cloud_to_xyz(self.latest_cloud)
        arrays = [_cloud_to_xyz(cloud) for cloud in self.cloud_buffer]
        arrays = [array for array in arrays if array.size > 0]
        if not arrays:
            return np.empty((0, 3), dtype=np.float64)
        return np.vstack(arrays)

    def save_background_cloud(self, path: Path) -> int:
        points = self._current_cloud_points()
        points = _crop_points(points, self.args)
        path.parent.mkdir(parents=True, exist_ok=True)
        np.save(path, points)
        return int(points.shape[0])

    def capture_scene(self, name: str) -> Scene:
        if self.latest_image is None:
            raise RuntimeError(f"no image received on {self.args.image_topic}")
        if self.latest_cloud is None and not self.args.print_camera_tag_only:
            raise RuntimeError(f"no PointCloud2 received on {self.args.cloud_topic}")

        camera_plane = _camera_plane_from_apriltag(
            self.latest_image,
            self.bridge,
            self.camera_matrix,
            self.dist_coeffs,
            self.args.tag_family,
            self.args.tag_id,
            self.args.tag_size_m,
        )
        if self.args.print_camera_tag_only:
            print(
                f"Camera AprilTag detected: family={self.args.tag_family} id={self.args.tag_id}; "
                f"plane_d={camera_plane.distance:.3f} m; "
                f"tag_center_camera=[{camera_plane.centroid[0]:.3f},"
                f"{camera_plane.centroid[1]:.3f},{camera_plane.centroid[2]:.3f}]"
            )
            raise BoundsOnlyRequested
        points = self._current_cloud_points()
        if self.args.print_cloud_bounds_only:
            cropped = _crop_points(points, self.args)
            print(f"Full cloud bounds: {_format_cloud_bounds(points)}")
            print(f"Crop points: {cropped.shape[0]}")
            raise BoundsOnlyRequested
        if self.args.print_roi_diagnostics_only:
            cropped = _crop_points(points, self.args)
            print(f"Full cloud bounds: {_format_cloud_bounds(points)}")
            print(f"Current crop points: {cropped.shape[0]}")
            print(_format_roi_diagnostics(points, camera_plane, self.args))
            raise BoundsOnlyRequested
        lidar_plane = _fit_lidar_plane(points, camera_plane, self.args)
        return Scene(
            name=name,
            image_stamp=_stamp_seconds(self.latest_image),
            cloud_stamp=_stamp_seconds(self.latest_cloud),
            camera_plane=camera_plane,
            lidar_plane=lidar_plane,
        )


def _parse_args() -> argparse.Namespace:
    pre_parser = argparse.ArgumentParser(add_help=False)
    pre_parser.add_argument(
        "--board-yaml",
        type=Path,
        default=Path("src/fast_livo2_ros2/config/box_tagstandard41h12_id0.yaml"),
    )
    pre_args, _ = pre_parser.parse_known_args()
    board = _load_board_yaml(pre_args.board_yaml)

    default_family = _board_value(board, "tag.family", "tag36h11")
    default_tag_id = _board_value(board, "tag.id", None)
    default_tag_size_m = _board_value(board, "tag.size_m", None)

    parser = argparse.ArgumentParser(parents=[pre_parser])
    parser.add_argument("--image-topic", default="/left_camera/image")
    parser.add_argument("--cloud-topic", default="/livox/lidar")
    parser.add_argument(
        "--camera-yaml",
        type=Path,
        default=Path("src/fast_livo2_ros2/config/camera_pinhole_rgb.yaml"),
    )
    parser.add_argument("--tag-family", default=default_family, choices=SUPPORTED_FAMILIES)
    parser.add_argument("--tag-id", type=int, default=default_tag_id)
    parser.add_argument("--tag-size-m", type=float, default=default_tag_size_m)
    parser.add_argument("--captures", type=int, default=8)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("calibration/lidar_camera_planes.yaml"),
    )

    parser.add_argument("--crop-x-min", type=float, default=-10.0)
    parser.add_argument("--crop-x-max", type=float, default=10.0)
    parser.add_argument("--crop-y-min", type=float, default=-10.0)
    parser.add_argument("--crop-y-max", type=float, default=10.0)
    parser.add_argument("--crop-z-min", type=float, default=-3.0)
    parser.add_argument("--crop-z-max", type=float, default=3.0)
    parser.add_argument("--print-cloud-bounds-only", action="store_true")
    parser.add_argument("--print-roi-diagnostics-only", action="store_true")
    parser.add_argument("--print-camera-tag-only", action="store_true")
    parser.add_argument("--zero-point-epsilon-m", type=float, default=0.03)
    parser.add_argument("--roi-range-window-m", type=float, default=1.0)
    parser.add_argument("--roi-voxel-size-m", type=float, default=0.20)
    parser.add_argument("--roi-voxel-limit", type=int, default=12)
    parser.add_argument("--save-background-npy", type=Path, default=None)
    parser.add_argument("--background-npy", type=Path, default=None)
    parser.add_argument("--background-distance-threshold-m", type=float, default=0.08)
    parser.add_argument("--background-max-points", type=int, default=60000)
    parser.add_argument("--cloud-accumulate-seconds", type=float, default=0.0)
    parser.add_argument("--cluster-foreground", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--foreground-cluster-voxel-m", type=float, default=0.10)
    parser.add_argument("--foreground-cluster-min-extent-m", type=float, default=0.08)
    parser.add_argument("--foreground-cluster-max-extent-m", type=float, default=1.10)
    parser.add_argument("--max-foreground-clusters-for-ransac", type=int, default=20)
    parser.add_argument("--print-foreground-clusters", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--print-foreground-cluster-limit", type=int, default=12)
    parser.add_argument("--plane-threshold-m", type=float, default=0.025)
    parser.add_argument("--min-lidar-plane-distance-m", type=float, default=0.2)
    parser.add_argument("--max-lidar-plane-distance-m", type=float, default=5.0)
    parser.add_argument("--max-camera-lidar-plane-distance-diff-m", type=float, default=0.8)
    parser.add_argument("--min-lidar-points", type=int, default=80)
    parser.add_argument("--max-ransac-points", type=int, default=5000)
    parser.add_argument("--max-plane-candidates", type=int, default=8)
    parser.add_argument("--candidate-remove-threshold-m", type=float, default=0.05)
    parser.add_argument("--min-board-extent-factor", type=float, default=0.25)
    parser.add_argument("--max-board-extent-factor", type=float, default=2.2)
    parser.add_argument("--candidate-extent-weight", type=float, default=1.0)
    parser.add_argument("--candidate-distance-weight", type=float, default=1.5)
    parser.add_argument("--candidate-inlier-weight", type=float, default=0.05)
    parser.add_argument("--normal-residual-weight", type=float, default=1.0)
    parser.add_argument("--distance-residual-weight", type=float, default=8.0)
    parser.add_argument("--centroid-residual-weight", type=float, default=4.0)
    parser.add_argument("--print-plane-candidates", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--print-plane-candidate-limit", type=int, default=5)
    parser.add_argument("--ransac-iterations", type=int, default=300)
    parser.add_argument("--ransac-seed", type=int, default=7)
    args = parser.parse_args()
    args.board_yaml = pre_args.board_yaml
    args.board = board
    args.background_points = None
    args.background_tree = None
    if args.tag_size_m is None:
        parser.error("--tag-size-m is required unless tag.size_m is present in --board-yaml")
    return args


def main() -> int:
    args = _parse_args()
    if args.background_npy is not None:
        args.background_points, args.background_tree = _load_background_points(args.background_npy, args)
    rclpy.init()
    node = CaptureNode(args)
    scenes: list[Scene] = []
    print("LiDAR-camera plane calibrator")
    if args.save_background_npy is not None:
        print("Background capture mode: remove the box/person from the LiDAR view, then press Enter.")
        print(f"Output background cloud: {args.save_background_npy}")
    elif args.background_npy is not None:
        print(
            f"Background subtraction enabled: {args.background_npy} "
            f"({args.background_points.shape[0]} points)"
        )
    print("Use a static board pose, then press Enter to capture. Type q + Enter to quit.")
    print(f"Need at least 3 scenes; target captures: {args.captures}")
    print(f"Output: {args.output}")

    try:
        if args.save_background_npy is not None:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
                if not sys.stdin.isatty():
                    time.sleep(0.1)
                    continue
                ready, _, _ = select.select([sys.stdin], [], [], 0.0)
                if not ready:
                    continue
                command = sys.stdin.readline().strip().lower()
                if command in {"q", "quit", "exit"}:
                    return 2
                count = node.save_background_cloud(args.save_background_npy)
                print(f"Saved background cloud: {args.save_background_npy} ({count} points)")
                return 0

        while rclpy.ok() and len(scenes) < args.captures:
            rclpy.spin_once(node, timeout_sec=0.1)
            if not sys.stdin.isatty():
                time.sleep(0.1)
                continue
            ready, _, _ = select.select([sys.stdin], [], [], 0.0)
            if not ready:
                continue
            command = sys.stdin.readline().strip().lower()
            if command in {"q", "quit", "exit"}:
                break
            name = f"scene_{len(scenes) + 1:02d}"
            try:
                scene = node.capture_scene(name)
            except BoundsOnlyRequested:
                return 0
            except Exception as exc:  # noqa: BLE001
                node.get_logger().error(f"{name} capture failed: {exc}")
                continue
            scenes.append(scene)
            print(
                f"{name}: camera d={scene.camera_plane.distance:.3f} m, "
                f"lidar d={scene.lidar_plane.distance:.3f} m, "
                f"lidar inliers={scene.lidar_plane.inliers}/{scene.lidar_plane.total}"
            )

            rcl = None
            pcl = None
            residuals = []
            if len(scenes) >= 3:
                rcl, pcl, residuals = _solve_lidar_to_camera(scenes, args)
                print("Current LiDAR->Camera estimate:")
                print("Rcl:")
                print(rcl)
                print(f"Pcl: {pcl}")
                worst_angle = max(abs(item["normal_angle_deg"]) for item in residuals)
                worst_dist = max(abs(item["distance_error_m"]) for item in residuals)
                print(f"worst residual: angle={worst_angle:.3f} deg, distance={worst_dist:.4f} m")
            _write_output(args.output, args.board, scenes, rcl, pcl, residuals)

        if len(scenes) < 3:
            print("Not enough scenes for extrinsic solve; saved captured plane data only.")
            _write_output(args.output, args.board, scenes, None, None, [])
            return 0 if args.captures < 3 else 2
        rcl, pcl, residuals = _solve_lidar_to_camera(scenes, args)
        _write_output(args.output, args.board, scenes, rcl, pcl, residuals)
        print(f"Final result written to {args.output}")
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
