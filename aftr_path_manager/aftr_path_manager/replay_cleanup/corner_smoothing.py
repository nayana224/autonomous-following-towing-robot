# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Corner detection and curve smoothing for replay-path cleanup."""

from __future__ import annotations

import math

from aftr_path_manager.path_types import RecordedPose
from aftr_path_manager.replay_cleanup.grid_utils import OccupancyGridSnapshot
from aftr_path_manager.replay_cleanup.grid_utils import is_pose_safe
from aftr_path_manager.replay_cleanup.path_context import PathContext
from aftr_path_manager.replay_cleanup.path_context import clamp
from aftr_path_manager.replay_cleanup.shift_stabilizer import build_lateral_shift_profile


def smooth_corner_segments(
    original_poses: list[RecordedPose],
    adjusted_poses: list[RecordedPose],
    path_contexts: list[PathContext | None],
    grid: OccupancyGridSnapshot,
    clearance_m: float,
    max_lateral_shift_m: float,
    corner_heading_threshold_rad: float,
    corner_clearance_delta_threshold_m: float,
    corner_shift_delta_threshold_m: float,
    corner_segment_padding_points: int,
    corner_curve_resample_step_m: float,
) -> list[RecordedPose]:
    """Curve-fit short corner segments so replay path turns more smoothly."""
    if len(adjusted_poses) < 5:
        return adjusted_poses

    corner_mask = build_corner_mask(
        path_contexts=path_contexts,
        original_poses=original_poses,
        adjusted_poses=adjusted_poses,
        lateral_shifts_m=None,
        corner_heading_threshold_rad=corner_heading_threshold_rad,
        corner_clearance_delta_threshold_m=corner_clearance_delta_threshold_m,
        corner_shift_delta_threshold_m=corner_shift_delta_threshold_m,
        padding_points=corner_segment_padding_points,
    )
    corner_segments = build_corner_segments(corner_mask)

    smoothed_poses = list(adjusted_poses)
    for start_index, end_index in corner_segments:
        if start_index <= 1 or end_index >= len(adjusted_poses) - 2:
            continue
        segment_poses = smooth_one_corner_segment(
            poses=smoothed_poses,
            start_index=start_index,
            end_index=end_index,
            resample_step_m=corner_curve_resample_step_m,
        )
        if segment_poses is None:
            continue

        for local_index, rebuilt_pose in enumerate(segment_poses, start=start_index):
            original_pose = original_poses[local_index]
            if math.hypot(
                rebuilt_pose.x - original_pose.x,
                rebuilt_pose.y - original_pose.y,
            ) > max_lateral_shift_m:
                continue
            if not is_pose_safe(rebuilt_pose, grid, clearance_m):
                continue
            smoothed_poses[local_index] = rebuilt_pose

    return smoothed_poses


def build_corner_mask(
    path_contexts: list[PathContext | None],
    original_poses: list[RecordedPose],
    adjusted_poses: list[RecordedPose],
    lateral_shifts_m: list[float] | None,
    corner_heading_threshold_rad: float,
    corner_clearance_delta_threshold_m: float,
    corner_shift_delta_threshold_m: float,
    padding_points: int,
) -> list[bool]:
    """Build one expanded corner mask for replay cleanup decisions.

    This helper centralizes corner-region detection so the strict
    ``corner_only`` policy can reuse the exact same mask before any candidate
    repair is attempted.
    """
    if lateral_shifts_m is None:
        lateral_shifts_m = build_lateral_shift_profile(
            original_poses=original_poses,
            adjusted_poses=adjusted_poses,
        )

    corner_mask = detect_corner_mask(
        path_contexts=path_contexts,
        lateral_shifts_m=lateral_shifts_m,
        corner_heading_threshold_rad=corner_heading_threshold_rad,
        corner_clearance_delta_threshold_m=corner_clearance_delta_threshold_m,
        corner_shift_delta_threshold_m=corner_shift_delta_threshold_m,
    )
    return expand_corner_mask(corner_mask, padding_points)


def detect_corner_mask(
    path_contexts: list[PathContext | None],
    lateral_shifts_m: list[float],
    corner_heading_threshold_rad: float,
    corner_clearance_delta_threshold_m: float,
    corner_shift_delta_threshold_m: float,
) -> list[bool]:
    """Detect corner candidates using heading, clearance, and shift-change cues."""
    corner_mask = [False] * len(path_contexts)
    for index in range(1, len(path_contexts) - 1):
        path_context = path_contexts[index]
        previous_context = path_contexts[index - 1]
        if path_context is None or path_context.context_name == "open_space":
            continue

        signal_count = 0
        if path_context.heading_change_rad >= corner_heading_threshold_rad:
            signal_count += 1

        if previous_context is not None:
            clearance_delta_m = max(
                abs(path_context.left_clearance_m - previous_context.left_clearance_m),
                abs(path_context.right_clearance_m - previous_context.right_clearance_m),
            )
            if clearance_delta_m >= corner_clearance_delta_threshold_m:
                signal_count += 1

        shift_delta_m = abs(lateral_shifts_m[index] - lateral_shifts_m[index - 1])
        if shift_delta_m >= corner_shift_delta_threshold_m:
            signal_count += 1

        if signal_count >= 2 or path_context.context_name == "cornering_to_corridor":
            corner_mask[index] = True

    return corner_mask


def expand_corner_mask(corner_mask: list[bool], padding_points: int) -> list[bool]:
    """Expand each detected corner point into a short corner segment window."""
    if padding_points <= 0:
        return list(corner_mask)

    expanded_mask = list(corner_mask)
    detected_indices = [index for index, value in enumerate(corner_mask) if value]
    for index in detected_indices:
        start_index = max(0, index - padding_points)
        end_index = min(len(expanded_mask) - 1, index + padding_points)
        for padded_index in range(start_index, end_index + 1):
            expanded_mask[padded_index] = True
    return expanded_mask


def build_corner_segments(corner_mask: list[bool]) -> list[tuple[int, int]]:
    """Convert a boolean corner mask into contiguous inclusive index ranges."""
    segments: list[tuple[int, int]] = []
    start_index: int | None = None
    for index, is_corner in enumerate(corner_mask):
        if is_corner and start_index is None:
            start_index = index
        elif not is_corner and start_index is not None:
            segments.append((start_index, index - 1))
            start_index = None
    if start_index is not None:
        segments.append((start_index, len(corner_mask) - 1))
    return segments


def smooth_one_corner_segment(
    poses: list[RecordedPose],
    start_index: int,
    end_index: int,
    resample_step_m: float,
) -> list[RecordedPose] | None:
    """Rebuild one corner segment with a short Catmull-Rom-like curve."""
    control_start = start_index - 1
    control_end = end_index + 1
    segment_points = [
        poses[control_start - 1],
        poses[control_start],
        poses[control_end],
        poses[control_end + 1],
    ]
    segment_length_m = estimate_segment_length(poses, control_start, control_end)
    if segment_length_m <= 1e-6:
        return None

    point_count = control_end - control_start + 1
    if point_count < 2:
        return None

    rebuilt_segment: list[RecordedPose] = []
    for point_index in range(point_count):
        ratio = point_index / max(1, point_count - 1)
        x_m, y_m = catmull_rom_point(segment_points, ratio)
        rebuilt_segment.append(
            RecordedPose(
                stamp_sec=poses[control_start + point_index].stamp_sec,
                frame_id=poses[control_start + point_index].frame_id,
                x=x_m,
                y=y_m,
                yaw=poses[control_start + point_index].yaw,
            )
        )

    rebuilt_segment = resample_corner_segment_if_needed(
        rebuilt_segment,
        original_count=point_count,
        resample_step_m=resample_step_m,
    )
    return rebuilt_segment[1:-1]


def estimate_segment_length(
    poses: list[RecordedPose],
    start_index: int,
    end_index: int,
) -> float:
    """Estimate one path segment length using the current replay polyline."""
    total_length_m = 0.0
    for previous_pose, current_pose in zip(
        poses[start_index:end_index],
        poses[start_index + 1 : end_index + 1],
    ):
        total_length_m += math.hypot(
            current_pose.x - previous_pose.x,
            current_pose.y - previous_pose.y,
        )
    return total_length_m


def catmull_rom_point(
    control_points: list[RecordedPose],
    ratio: float,
) -> tuple[float, float]:
    """Evaluate a uniform Catmull-Rom point between the middle control points."""
    p0, p1, p2, p3 = control_points
    t = clamp(ratio, 0.0, 1.0)
    t2 = t * t
    t3 = t2 * t
    x_m = 0.5 * (
        (2.0 * p1.x)
        + (-p0.x + p2.x) * t
        + (2.0 * p0.x - 5.0 * p1.x + 4.0 * p2.x - p3.x) * t2
        + (-p0.x + 3.0 * p1.x - 3.0 * p2.x + p3.x) * t3
    )
    y_m = 0.5 * (
        (2.0 * p1.y)
        + (-p0.y + p2.y) * t
        + (2.0 * p0.y - 5.0 * p1.y + 4.0 * p2.y - p3.y) * t2
        + (-p0.y + 3.0 * p1.y - 3.0 * p2.y + p3.y) * t3
    )
    return x_m, y_m


def resample_corner_segment_if_needed(
    poses: list[RecordedPose],
    original_count: int,
    resample_step_m: float,
) -> list[RecordedPose]:
    """Resample one rebuilt corner curve while preserving the original point count."""
    if len(poses) < 2 or resample_step_m <= 0.0:
        return poses

    total_length_m = estimate_segment_length(poses, 0, len(poses) - 1)
    if total_length_m <= 1e-6:
        return poses

    sampled_points: list[tuple[float, float]] = [(poses[0].x, poses[0].y)]
    travelled_m = 0.0
    target_distance_m = resample_step_m
    for previous_pose, current_pose in zip(poses[:-1], poses[1:]):
        dx_m = current_pose.x - previous_pose.x
        dy_m = current_pose.y - previous_pose.y
        segment_length_m = math.hypot(dx_m, dy_m)
        if segment_length_m <= 1e-6:
            continue

        while travelled_m + segment_length_m >= target_distance_m:
            ratio = (target_distance_m - travelled_m) / segment_length_m
            sampled_points.append(
                (
                    previous_pose.x + dx_m * ratio,
                    previous_pose.y + dy_m * ratio,
                )
            )
            target_distance_m += resample_step_m
        travelled_m += segment_length_m

    sampled_points.append((poses[-1].x, poses[-1].y))
    source_points = sampled_points

    rebuilt_poses: list[RecordedPose] = []
    for point_index in range(original_count):
        source_ratio = point_index / max(1, original_count - 1)
        source_position = source_ratio * (len(source_points) - 1)
        lower_index = int(math.floor(source_position))
        upper_index = min(len(source_points) - 1, lower_index + 1)
        local_ratio = source_position - lower_index
        lower_x_m, lower_y_m = source_points[lower_index]
        upper_x_m, upper_y_m = source_points[upper_index]
        rebuilt_poses.append(
            RecordedPose(
                stamp_sec=poses[min(point_index, len(poses) - 1)].stamp_sec,
                frame_id=poses[min(point_index, len(poses) - 1)].frame_id,
                x=lower_x_m + (upper_x_m - lower_x_m) * local_ratio,
                y=lower_y_m + (upper_y_m - lower_y_m) * local_ratio,
                yaw=poses[min(point_index, len(poses) - 1)].yaw,
            )
        )
    return rebuilt_poses
