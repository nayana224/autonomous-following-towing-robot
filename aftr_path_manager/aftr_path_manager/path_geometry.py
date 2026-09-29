# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Geometry helpers for path recording and replay."""

from __future__ import annotations

import math

from aftr_path_manager.path_types import RecordedPose


def point_to_segment_distance(
    point: RecordedPose,
    segment_start: RecordedPose,
    segment_end: RecordedPose,
) -> float:
    """Return the distance from a point to a 2D line segment."""
    start_x = segment_start.x
    start_y = segment_start.y
    end_x = segment_end.x
    end_y = segment_end.y
    point_x = point.x
    point_y = point.y

    segment_dx = end_x - start_x
    segment_dy = end_y - start_y
    segment_length_sq = segment_dx * segment_dx + segment_dy * segment_dy
    if segment_length_sq <= 1e-9:
        return math.hypot(point_x - start_x, point_y - start_y)

    ratio = (
        ((point_x - start_x) * segment_dx + (point_y - start_y) * segment_dy)
        / segment_length_sq
    )
    ratio = max(0.0, min(1.0, ratio))
    projection_x = start_x + ratio * segment_dx
    projection_y = start_y + ratio * segment_dy
    return math.hypot(point_x - projection_x, point_y - projection_y)


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Return yaw from a quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """Return a quaternion tuple from yaw."""
    half_yaw = yaw * 0.5
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


def normalize_angle(angle: float) -> float:
    """Normalize an angle to the ``[-pi, pi]`` range."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def nearest_pose_index(poses: list[RecordedPose], pose: RecordedPose) -> int:
    """Return the index of the path pose closest to ``pose``."""
    return min(
        range(len(poses)),
        key=lambda index: math.hypot(
            poses[index].x - pose.x,
            poses[index].y - pose.y,
        ),
    )
