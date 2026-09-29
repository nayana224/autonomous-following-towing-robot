# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
"""Endpoint-aware safe smoothing for continuous clearance correction.

This module keeps the segment-direction behavior from
``continuous_clearance_cleanup`` and only improves the final entry/exit shape.
Every candidate change is accepted only when the configured clearance remains
valid.
"""

from __future__ import annotations

import math

from aftr_path_manager.continuous_clearance_cleanup import (
    cleanup_path_points as continuous_cleanup_path_points,
)
from aftr_path_manager.path_types import RecordedPose
from aftr_path_manager.replay_cleanup.grid_utils import OccupancyGridSnapshot
from aftr_path_manager.replay_cleanup.grid_utils import is_pose_safe


ENDPOINT_RAMP_DISTANCE_M = 1.00
SMOOTHING_NEIGHBOR_WEIGHT = 0.25
POSITION_CHANGE_EPSILON_M = 0.002


def cleanup_path_points(
    poses: list[RecordedPose],
    grid: OccupancyGridSnapshot,
    clearance_m: float,
    max_lateral_shift_m: float,
    lateral_sample_step_m: float,
    preserve_start_distance_m: float,
    preserve_goal_distance_m: float,
    correction_mode: str,
    centering_enabled: bool,
    corridor_max_width_m: float,
    corner_heading_threshold_rad: float,
    corner_shift_scale: float,
    smoothing_window: int,
    transition_distance_m: float,
    max_shift_delta_m: float,
    shift_smoothing_window: int,
    outlier_threshold_m: float,
    pre_simplify_enabled: bool,
    pre_simplify_min_spacing_m: float,
    pre_simplify_heading_threshold_rad: float,
    corner_clearance_delta_threshold_m: float,
    corner_shift_delta_threshold_m: float,
    corner_segment_padding_points: int,
    corner_curve_resample_step_m: float,
    corner_entry_transition_distance_m: float = 0.60,
    corner_exit_transition_distance_m: float = 0.25,
    corner_smoothing_strength: float = 0.65,
    corner_max_deviation_m: float = 0.20,
    preferred_clearance_m: float = 0.0,
    preferred_max_lateral_shift_m: float = 0.35,
):
    """Apply continuous correction, then smooth safe endpoint transitions."""
    result = continuous_cleanup_path_points(
        poses=poses,
        grid=grid,
        clearance_m=clearance_m,
        max_lateral_shift_m=max_lateral_shift_m,
        lateral_sample_step_m=lateral_sample_step_m,
        preserve_start_distance_m=preserve_start_distance_m,
        preserve_goal_distance_m=preserve_goal_distance_m,
        correction_mode=correction_mode,
        centering_enabled=centering_enabled,
        corridor_max_width_m=corridor_max_width_m,
        corner_heading_threshold_rad=corner_heading_threshold_rad,
        corner_shift_scale=corner_shift_scale,
        smoothing_window=smoothing_window,
        transition_distance_m=transition_distance_m,
        max_shift_delta_m=max_shift_delta_m,
        shift_smoothing_window=shift_smoothing_window,
        outlier_threshold_m=outlier_threshold_m,
        pre_simplify_enabled=pre_simplify_enabled,
        pre_simplify_min_spacing_m=pre_simplify_min_spacing_m,
        pre_simplify_heading_threshold_rad=pre_simplify_heading_threshold_rad,
        corner_clearance_delta_threshold_m=corner_clearance_delta_threshold_m,
        corner_shift_delta_threshold_m=corner_shift_delta_threshold_m,
        corner_segment_padding_points=corner_segment_padding_points,
        corner_curve_resample_step_m=corner_curve_resample_step_m,
        corner_entry_transition_distance_m=corner_entry_transition_distance_m,
        corner_exit_transition_distance_m=corner_exit_transition_distance_m,
        corner_smoothing_strength=corner_smoothing_strength,
        corner_max_deviation_m=corner_max_deviation_m,
        preferred_clearance_m=preferred_clearance_m,
        preferred_max_lateral_shift_m=preferred_max_lateral_shift_m,
    )

    if len(poses) < 3 or len(result.poses) != len(poses):
        return result

    corrected = _apply_endpoint_shift_envelope(
        original_poses=poses,
        corrected_poses=result.poses,
        grid=grid,
        clearance_m=clearance_m,
        preserve_start_distance_m=preserve_start_distance_m,
        preserve_goal_distance_m=preserve_goal_distance_m,
    )
    corrected = _safe_weighted_smoothing(
        original_poses=poses,
        corrected_poses=corrected,
        grid=grid,
        clearance_m=clearance_m,
        passes=smoothing_window,
        smoothing_strength=corner_smoothing_strength,
        max_deviation_m=corner_max_deviation_m,
    )
    corrected = _recompute_yaws(corrected)

    result.poses = corrected
    result.output_points = len(corrected)
    result.correction_mode = "endpoint_smoothed_continuous_clearance"
    return result


def _apply_endpoint_shift_envelope(
    original_poses: list[RecordedPose],
    corrected_poses: list[RecordedPose],
    grid: OccupancyGridSnapshot,
    clearance_m: float,
    preserve_start_distance_m: float,
    preserve_goal_distance_m: float,
) -> list[RecordedPose]:
    """Reduce lateral shift gradually near the preserved start and goal areas."""
    cumulative = _cumulative_distances(original_poses)
    total_distance = cumulative[-1]
    output = list(corrected_poses)

    start_anchor = max(0.0, float(preserve_start_distance_m))
    goal_anchor = max(0.0, float(preserve_goal_distance_m))
    ramp_distance = max(0.1, ENDPOINT_RAMP_DISTANCE_M)

    for index, (original, corrected) in enumerate(
        zip(original_poses, corrected_poses)
    ):
        distance_from_start = cumulative[index]
        distance_to_goal = total_distance - cumulative[index]

        start_factor = _endpoint_factor(
            distance_from_start,
            start_anchor,
            ramp_distance,
        )
        goal_factor = _endpoint_factor(
            distance_to_goal,
            goal_anchor,
            ramp_distance,
        )
        factor = min(start_factor, goal_factor)

        if factor >= 1.0 - 1e-9:
            continue

        candidate = _interpolate_pose(original, corrected, factor)
        if is_pose_safe(candidate, grid, clearance_m):
            output[index] = candidate

    return output


def _endpoint_factor(distance: float, preserve_distance: float, ramp_distance: float) -> float:
    """Return a smooth 0..1 shift factor beyond a preserved endpoint region."""
    if distance <= preserve_distance:
        return 0.0
    normalized = (distance - preserve_distance) / ramp_distance
    normalized = max(0.0, min(1.0, normalized))
    return normalized * normalized * (3.0 - 2.0 * normalized)


def _safe_weighted_smoothing(
    original_poses: list[RecordedPose],
    corrected_poses: list[RecordedPose],
    grid: OccupancyGridSnapshot,
    clearance_m: float,
    passes: int,
    smoothing_strength: float,
    max_deviation_m: float,
) -> list[RecordedPose]:
    """Smooth coordinates while retaining only clearance-safe candidates."""
    smoothed = list(corrected_poses)
    strength = max(0.0, min(1.0, float(smoothing_strength)))
    neighbor_weight = SMOOTHING_NEIGHBOR_WEIGHT * strength
    center_weight = 1.0 - 2.0 * neighbor_weight

    for _ in range(max(0, int(passes))):
        next_pass = list(smoothed)
        for index in range(1, len(smoothed) - 1):
            current = smoothed[index]
            previous = smoothed[index - 1]
            following = smoothed[index + 1]

            candidate = RecordedPose(
                stamp_sec=current.stamp_sec,
                frame_id=current.frame_id,
                x=(
                    previous.x * neighbor_weight
                    + current.x * center_weight
                    + following.x * neighbor_weight
                ),
                y=(
                    previous.y * neighbor_weight
                    + current.y * center_weight
                    + following.y * neighbor_weight
                ),
                yaw=current.yaw,
            )
            candidate = _limit_deviation(
                original_poses[index],
                candidate,
                max_deviation_m,
            )

            movement = math.hypot(
                candidate.x - current.x,
                candidate.y - current.y,
            )
            if movement <= POSITION_CHANGE_EPSILON_M:
                continue

            if is_pose_safe(candidate, grid, clearance_m):
                next_pass[index] = candidate
                continue

            original = original_poses[index]
            halfway = _interpolate_pose(original, candidate, 0.5)
            if is_pose_safe(halfway, grid, clearance_m):
                next_pass[index] = halfway

        smoothed = next_pass

    return smoothed


def _limit_deviation(
    original: RecordedPose,
    candidate: RecordedPose,
    maximum_deviation_m: float,
) -> RecordedPose:
    """Limit smoothing displacement while preserving safety corrections."""
    dx_m = candidate.x - original.x
    dy_m = candidate.y - original.y
    distance_m = math.hypot(dx_m, dy_m)
    limit_m = max(0.0, float(maximum_deviation_m))
    if distance_m <= limit_m or distance_m <= 1e-9:
        return candidate
    ratio = limit_m / distance_m
    return RecordedPose(
        stamp_sec=candidate.stamp_sec,
        frame_id=candidate.frame_id,
        x=original.x + dx_m * ratio,
        y=original.y + dy_m * ratio,
        yaw=candidate.yaw,
    )


def _interpolate_pose(
    start: RecordedPose,
    end: RecordedPose,
    fraction: float,
) -> RecordedPose:
    fraction = max(0.0, min(1.0, fraction))
    return RecordedPose(
        stamp_sec=end.stamp_sec,
        frame_id=end.frame_id,
        x=start.x + (end.x - start.x) * fraction,
        y=start.y + (end.y - start.y) * fraction,
        yaw=end.yaw,
    )


def _cumulative_distances(poses: list[RecordedPose]) -> list[float]:
    distances = [0.0]
    for previous, current in zip(poses[:-1], poses[1:]):
        distances.append(
            distances[-1]
            + math.hypot(current.x - previous.x, current.y - previous.y)
        )
    return distances


def _recompute_yaws(poses: list[RecordedPose]) -> list[RecordedPose]:
    if len(poses) < 2:
        return list(poses)

    output: list[RecordedPose] = []
    for index, pose in enumerate(poses):
        if index == len(poses) - 1:
            previous = poses[index - 1]
            yaw = math.atan2(pose.y - previous.y, pose.x - previous.x)
        else:
            following = poses[index + 1]
            yaw = math.atan2(following.y - pose.y, following.x - pose.x)
        output.append(
            RecordedPose(
                stamp_sec=pose.stamp_sec,
                frame_id=pose.frame_id,
                x=pose.x,
                y=pose.y,
                yaw=yaw,
            )
        )
    return output
