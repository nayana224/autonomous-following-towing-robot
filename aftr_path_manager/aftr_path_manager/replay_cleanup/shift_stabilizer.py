# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Lateral-shift stabilization helpers for replay-path cleanup."""

from __future__ import annotations

import math

from aftr_path_manager.path_types import RecordedPose
from aftr_path_manager.replay_cleanup.grid_utils import OccupancyGridSnapshot
from aftr_path_manager.replay_cleanup.grid_utils import is_pose_safe
from aftr_path_manager.replay_cleanup.path_context import PathContext
from aftr_path_manager.replay_cleanup.path_context import clamp
from aftr_path_manager.replay_cleanup.path_context import infer_local_path_yaw


def stabilize_lateral_shift_profile(
    original_poses: list[RecordedPose],
    adjusted_poses: list[RecordedPose],
    path_contexts: list[PathContext | None],
    grid: OccupancyGridSnapshot,
    clearance_m: float,
    max_lateral_shift_m: float,
    max_shift_delta_m: float,
    shift_smoothing_window: int,
    outlier_threshold_m: float,
) -> list[RecordedPose]:
    """Reduce spike-like lateral artifacts after corridor centering."""
    if len(adjusted_poses) < 3:
        return adjusted_poses

    lateral_shifts_m = build_lateral_shift_profile(
        original_poses=original_poses,
        adjusted_poses=adjusted_poses,
    )
    lateral_shifts_m = limit_lateral_shift_outliers(
        lateral_shifts_m=lateral_shifts_m,
        path_contexts=path_contexts,
        outlier_threshold_m=outlier_threshold_m,
    )
    lateral_shifts_m = limit_lateral_shift_delta(
        lateral_shifts_m=lateral_shifts_m,
        path_contexts=path_contexts,
        max_shift_delta_m=max_shift_delta_m,
    )
    lateral_shifts_m = smooth_lateral_shift_profile(
        lateral_shifts_m=lateral_shifts_m,
        path_contexts=path_contexts,
        smoothing_window=shift_smoothing_window,
    )
    return rebuild_path_from_lateral_shifts(
        original_poses=original_poses,
        reference_poses=adjusted_poses,
        lateral_shifts_m=lateral_shifts_m,
        path_contexts=path_contexts,
        grid=grid,
        clearance_m=clearance_m,
        max_lateral_shift_m=max_lateral_shift_m,
    )


def build_lateral_shift_profile(
    original_poses: list[RecordedPose],
    adjusted_poses: list[RecordedPose],
) -> list[float]:
    """Project adjusted replay points onto the original local normal direction."""
    shifts_m = [0.0] * len(adjusted_poses)
    for index in range(1, len(adjusted_poses) - 1):
        tangent_yaw = infer_local_path_yaw(
            original_poses[index - 1],
            original_poses[index],
            original_poses[index + 1],
        )
        normal_x = -math.sin(tangent_yaw)
        normal_y = math.cos(tangent_yaw)
        dx_m = adjusted_poses[index].x - original_poses[index].x
        dy_m = adjusted_poses[index].y - original_poses[index].y
        shifts_m[index] = dx_m * normal_x + dy_m * normal_y
    return shifts_m


def limit_lateral_shift_outliers(
    lateral_shifts_m: list[float],
    path_contexts: list[PathContext | None],
    outlier_threshold_m: float,
) -> list[float]:
    """Clamp single-point spikes against neighboring lateral shifts."""
    if outlier_threshold_m <= 0.0:
        return list(lateral_shifts_m)

    limited_shifts_m = list(lateral_shifts_m)
    for index in range(1, len(lateral_shifts_m) - 1):
        path_context = path_contexts[index]
        if path_context is None or path_context.context_name == "open_space":
            continue

        neighbor_mean_m = (
            lateral_shifts_m[index - 1] + lateral_shifts_m[index + 1]
        ) * 0.5
        delta_from_neighbors_m = lateral_shifts_m[index] - neighbor_mean_m
        if abs(delta_from_neighbors_m) <= outlier_threshold_m:
            continue

        limited_shifts_m[index] = (
            neighbor_mean_m
            + math.copysign(outlier_threshold_m, delta_from_neighbors_m)
        )
    return limited_shifts_m


def limit_lateral_shift_delta(
    lateral_shifts_m: list[float],
    path_contexts: list[PathContext | None],
    max_shift_delta_m: float,
) -> list[float]:
    """Limit how quickly lateral shift can change between adjacent points."""
    if max_shift_delta_m <= 0.0:
        return list(lateral_shifts_m)

    limited_shifts_m = list(lateral_shifts_m)
    for index in range(1, len(limited_shifts_m)):
        current_context = path_contexts[index]
        previous_context = path_contexts[index - 1]
        if (
            (current_context is None or current_context.context_name == "open_space")
            and (
                previous_context is None
                or previous_context.context_name == "open_space"
            )
        ):
            continue

        delta_m = limited_shifts_m[index] - limited_shifts_m[index - 1]
        if delta_m > max_shift_delta_m:
            limited_shifts_m[index] = limited_shifts_m[index - 1] + max_shift_delta_m
        elif delta_m < -max_shift_delta_m:
            limited_shifts_m[index] = limited_shifts_m[index - 1] - max_shift_delta_m
    return limited_shifts_m


def smooth_lateral_shift_profile(
    lateral_shifts_m: list[float],
    path_contexts: list[PathContext | None],
    smoothing_window: int,
) -> list[float]:
    """Apply a small moving average to the corridor-only lateral shift profile."""
    if smoothing_window <= 0:
        return list(lateral_shifts_m)

    smoothed_shifts_m = list(lateral_shifts_m)
    for index in range(1, len(lateral_shifts_m) - 1):
        path_context = path_contexts[index]
        if path_context is None or path_context.context_name == "open_space":
            continue

        start_index = max(0, index - smoothing_window)
        end_index = min(len(lateral_shifts_m) - 1, index + smoothing_window)
        samples = lateral_shifts_m[start_index : end_index + 1]
        smoothed_shifts_m[index] = sum(samples) / len(samples)
    return smoothed_shifts_m


def rebuild_path_from_lateral_shifts(
    original_poses: list[RecordedPose],
    reference_poses: list[RecordedPose],
    lateral_shifts_m: list[float],
    path_contexts: list[PathContext | None],
    grid: OccupancyGridSnapshot,
    clearance_m: float,
    max_lateral_shift_m: float,
) -> list[RecordedPose]:
    """Rebuild replay poses from the stabilized lateral shift profile."""
    rebuilt_poses = list(reference_poses)
    for index in range(1, len(reference_poses) - 1):
        path_context = path_contexts[index]
        if path_context is None or path_context.context_name == "open_space":
            continue

        tangent_yaw = infer_local_path_yaw(
            original_poses[index - 1],
            original_poses[index],
            original_poses[index + 1],
        )
        normal_x = -math.sin(tangent_yaw)
        normal_y = math.cos(tangent_yaw)
        shift_m = clamp(
            lateral_shifts_m[index],
            -max_lateral_shift_m,
            max_lateral_shift_m,
        )
        rebuilt_pose = RecordedPose(
            stamp_sec=reference_poses[index].stamp_sec,
            frame_id=reference_poses[index].frame_id,
            x=original_poses[index].x + normal_x * shift_m,
            y=original_poses[index].y + normal_y * shift_m,
            yaw=reference_poses[index].yaw,
        )
        if not is_pose_safe(rebuilt_pose, grid, clearance_m):
            continue
        rebuilt_poses[index] = rebuilt_pose
    return rebuilt_poses


def smooth_corridor_segments(
    original_poses: list[RecordedPose],
    adjusted_poses: list[RecordedPose],
    path_contexts: list[PathContext | None],
    grid: OccupancyGridSnapshot,
    clearance_m: float,
    max_lateral_shift_m: float,
    smoothing_window: int,
) -> list[RecordedPose]:
    """Apply a light moving-average smoothing pass to corridor-centered segments."""
    if smoothing_window <= 0 or len(adjusted_poses) < 3:
        return adjusted_poses

    smoothed_poses = list(adjusted_poses)
    for index in range(1, len(adjusted_poses) - 1):
        path_context = path_contexts[index]
        if path_context is None or path_context.context_name == "open_space":
            continue

        start_index = max(0, index - smoothing_window)
        end_index = min(len(adjusted_poses) - 1, index + smoothing_window)
        sample_poses = adjusted_poses[start_index : end_index + 1]
        mean_x_m = sum(pose.x for pose in sample_poses) / len(sample_poses)
        mean_y_m = sum(pose.y for pose in sample_poses) / len(sample_poses)

        original_pose = original_poses[index]
        smoothed_pose = RecordedPose(
            stamp_sec=adjusted_poses[index].stamp_sec,
            frame_id=adjusted_poses[index].frame_id,
            x=mean_x_m,
            y=mean_y_m,
            yaw=adjusted_poses[index].yaw,
        )
        if math.hypot(
            smoothed_pose.x - original_pose.x,
            smoothed_pose.y - original_pose.y,
        ) > max_lateral_shift_m:
            continue
        if not is_pose_safe(smoothed_pose, grid, clearance_m):
            continue

        smoothed_poses[index] = smoothed_pose

    return smoothed_poses


def apply_boundary_transition(
    original_poses: list[RecordedPose],
    adjusted_poses: list[RecordedPose],
    path_contexts: list[PathContext | None],
    grid: OccupancyGridSnapshot,
    clearance_m: float,
    max_lateral_shift_m: float,
    cumulative_distances: list[float],
    total_distance_m: float,
    preserve_start_distance_m: float,
    preserve_goal_distance_m: float,
    transition_distance_m: float,
) -> list[RecordedPose]:
    """Blend corridor centering in and out near the path boundaries."""
    if transition_distance_m <= 0.0:
        return adjusted_poses

    transitioned_poses = list(adjusted_poses)
    for index in range(1, len(adjusted_poses) - 1):
        path_context = path_contexts[index]
        if path_context is None or path_context.context_name == "open_space":
            continue

        distance_from_start_m = cumulative_distances[index]
        distance_to_goal_m = total_distance_m - distance_from_start_m
        blend_weight = 1.0

        if distance_from_start_m < preserve_start_distance_m + transition_distance_m:
            start_progress = (
                distance_from_start_m - preserve_start_distance_m
            ) / transition_distance_m
            blend_weight = min(blend_weight, clamp(start_progress, 0.0, 1.0))

        if distance_to_goal_m < preserve_goal_distance_m + transition_distance_m:
            goal_progress = (
                distance_to_goal_m - preserve_goal_distance_m
            ) / transition_distance_m
            blend_weight = min(blend_weight, clamp(goal_progress, 0.0, 1.0))

        if blend_weight >= 0.999:
            continue

        original_pose = original_poses[index]
        adjusted_pose = adjusted_poses[index]
        blended_pose = RecordedPose(
            stamp_sec=adjusted_pose.stamp_sec,
            frame_id=adjusted_pose.frame_id,
            x=original_pose.x + (adjusted_pose.x - original_pose.x) * blend_weight,
            y=original_pose.y + (adjusted_pose.y - original_pose.y) * blend_weight,
            yaw=adjusted_pose.yaw,
        )
        if math.hypot(
            blended_pose.x - original_pose.x,
            blended_pose.y - original_pose.y,
        ) > max_lateral_shift_m:
            continue
        if not is_pose_safe(blended_pose, grid, clearance_m):
            continue

        transitioned_poses[index] = blended_pose

    return transitioned_poses
