# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
"""Helpers for recorded-path filtering and simplification."""

from __future__ import annotations

import math

from aftr_path_manager.path_geometry import normalize_angle
from aftr_path_manager.path_geometry import point_to_segment_distance
from aftr_path_manager.path_types import RecordedPose


def should_record_pose(
    pose: RecordedPose,
    last_recorded_pose: RecordedPose | None,
    min_distance_m: float,
    min_yaw_delta_rad: float,
    min_time_delta_s: float,
) -> bool:
    """Return True when a new pose should be appended to the recorded path."""
    if last_recorded_pose is None:
        return True

    distance = math.hypot(
        pose.x - last_recorded_pose.x,
        pose.y - last_recorded_pose.y,
    )
    yaw_delta = abs(normalize_angle(pose.yaw - last_recorded_pose.yaw))
    time_delta = pose.stamp_sec - last_recorded_pose.stamp_sec

    if time_delta < float(min_time_delta_s):
        return False

    return distance >= float(min_distance_m) or yaw_delta >= float(min_yaw_delta_rad)


def filter_global_frame_poses(
    poses: list[RecordedPose],
    global_frame: str,
) -> list[RecordedPose]:
    """Return only poses that belong to the expected global frame."""
    return [pose for pose in poses if pose.frame_id == global_frame]


def simplify_recorded_path(
    poses: list[RecordedPose],
    simplify_tolerance_m: float,
    min_yaw_delta_rad: float,
) -> list[RecordedPose]:
    """Remove nearly collinear middle points from a recorded path."""
    if len(poses) <= 2 or float(simplify_tolerance_m) <= 0.0:
        return list(poses)

    simplified = [poses[0]]
    for index in range(1, len(poses) - 1):
        previous_pose = simplified[-1]
        current_pose = poses[index]
        next_pose = poses[index + 1]
        yaw_delta = abs(normalize_angle(next_pose.yaw - previous_pose.yaw))
        distance_from_line = point_to_segment_distance(
            current_pose,
            previous_pose,
            next_pose,
        )

        if (
            yaw_delta < float(min_yaw_delta_rad)
            and distance_from_line < float(simplify_tolerance_m)
        ):
            continue

        simplified.append(current_pose)

    simplified.append(poses[-1])
    return simplified
