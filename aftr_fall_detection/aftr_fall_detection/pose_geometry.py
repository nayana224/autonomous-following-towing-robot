# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
"""Dependency-light pose geometry used by the fall detector."""

import math

import numpy as np


HEAD = (0, 1, 2, 3, 4)
SHOULDERS = (5, 6)
HIPS = (11, 12)
LOWER_BODY = (13, 14, 15, 16)


def analyze_pose_geometry(
    bbox: np.ndarray,
    keypoints: np.ndarray,
    *,
    confidence_threshold: float,
    full_angle_threshold: float,
    partial_angle_threshold: float,
    full_body_aspect_threshold: float,
    partial_upper_body_aspect_threshold: float,
) -> dict:
    """Classify pose geometry without requiring a complete body silhouette."""
    x1, y1, x2, y2 = bbox.astype(int)
    box_width = max(float(x2 - x1), 1.0)
    box_height = max(float(y2 - y1), 1.0)
    box_aspect_ratio = box_width / box_height

    head_points = _valid_points(keypoints, HEAD, confidence_threshold)
    shoulders = _valid_points(keypoints, SHOULDERS, confidence_threshold)
    hips = _valid_points(keypoints, HIPS, confidence_threshold)
    lower_body_points = _valid_points(
        keypoints,
        LOWER_BODY,
        confidence_threshold,
    )

    if not (head_points and shoulders and hips):
        return {
            "bbox": (x1, y1, x2, y2),
            "partial_pose": True,
            "partial_reason": "UPPER_BODY_INCOMPLETE",
            "visibility": "AMBIGUOUS",
            "fall_candidate": False,
            "torso_angle_deg": 0.0,
            "box_aspect_ratio": box_aspect_ratio,
            "torso_height_ratio": 1.0,
            "body_aspect_ratio": 0.0,
            "upper_body_aspect_ratio": 0.0,
            "evidence_count": 0,
            "evidence_total": 2,
        }

    shoulder_center = _mean_point(shoulders)
    hip_center = _mean_point(hips)
    torso_dx = hip_center[0] - shoulder_center[0]
    torso_dy = hip_center[1] - shoulder_center[1]
    torso_angle_deg = math.degrees(
        math.atan2(abs(torso_dx), max(abs(torso_dy), 1e-6))
    )
    torso_height_ratio = abs(torso_dy) / box_height

    upper_body_points = head_points + shoulders + hips
    upper_body_aspect_ratio = _point_cloud_aspect(upper_body_points)
    body_aspect_ratio = _point_cloud_aspect(
        upper_body_points + lower_body_points
    )
    visibility = "FULL" if len(lower_body_points) >= 2 else "PARTIAL"

    horizontal_torso = torso_angle_deg >= (
        full_angle_threshold
        if visibility == "FULL"
        else partial_angle_threshold
    )
    supporting_shape = (
        body_aspect_ratio >= full_body_aspect_threshold
        if visibility == "FULL"
        else upper_body_aspect_ratio >= partial_upper_body_aspect_threshold
    )

    return {
        "bbox": (x1, y1, x2, y2),
        "partial_pose": False,
        "partial_reason": "",
        "visibility": visibility,
        "fall_candidate": horizontal_torso and supporting_shape,
        "torso_angle_deg": torso_angle_deg,
        "box_aspect_ratio": box_aspect_ratio,
        "torso_height_ratio": torso_height_ratio,
        "body_aspect_ratio": body_aspect_ratio,
        "upper_body_aspect_ratio": upper_body_aspect_ratio,
        "evidence_count": int(horizontal_torso) + int(supporting_shape),
        "evidence_total": 2,
    }


def _valid_points(
    keypoints: np.ndarray,
    indices: tuple[int, ...],
    confidence_threshold: float,
) -> list[np.ndarray]:
    return [
        keypoints[index]
        for index in indices
        if (
            len(keypoints[index]) >= 3
            and float(keypoints[index][2]) >= confidence_threshold
        )
    ]


def _mean_point(points: list[np.ndarray]) -> tuple[float, float]:
    return (
        sum(float(point[0]) for point in points) / len(points),
        sum(float(point[1]) for point in points) / len(points),
    )


def _point_cloud_aspect(points: list[np.ndarray]) -> float:
    xs = [float(point[0]) for point in points]
    ys = [float(point[1]) for point in points]
    return (max(xs) - min(xs)) / max(max(ys) - min(ys), 1.0)
