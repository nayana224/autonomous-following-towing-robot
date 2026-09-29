# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Facade for replay-path cleanup and corridor-centering helpers."""

from __future__ import annotations

from dataclasses import dataclass
import math

from aftr_path_manager.path_types import RecordedPose
from aftr_path_manager.replay_cleanup.candidate_search import find_best_lateral_candidate
from aftr_path_manager.replay_cleanup.corner_smoothing import build_corner_mask
from aftr_path_manager.replay_cleanup.corner_smoothing import smooth_corner_segments
from aftr_path_manager.replay_cleanup.grid_utils import OccupancyGridSnapshot
from aftr_path_manager.replay_cleanup.grid_utils import build_grid_snapshot
from aftr_path_manager.replay_cleanup.grid_utils import is_pose_safe
from aftr_path_manager.replay_cleanup.path_context import PathContext
from aftr_path_manager.replay_cleanup.path_context import classify_path_context
from aftr_path_manager.replay_cleanup.path_context import compute_heading_change
from aftr_path_manager.replay_cleanup.shift_stabilizer import apply_boundary_transition
from aftr_path_manager.replay_cleanup.shift_stabilizer import smooth_corridor_segments
from aftr_path_manager.replay_cleanup.shift_stabilizer import stabilize_lateral_shift_profile


@dataclass
class PathCleanupResult:
    """Path cleanup output and small audit counters."""

    poses: list[RecordedPose]
    correction_mode: str
    input_points: int
    output_points: int
    open_space_points: int
    corridor_points: int
    cornering_points: int
    corner_mask_count: int
    preserved_safe_points: int
    preserved_outside_corner_points: int
    repaired_unsafe_points: int
    corner_adjusted_points: int
    adjusted_inside_corner_points: int
    adjusted_points: int
    unresolved_inside_corner_points: int
    unresolved_points: int
    unsafe_outside_corner_ignored_points: int
    adjusted_indices: list[int]


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
) -> PathCleanupResult:
    """Run the full replay-path cleanup pipeline on recorded poses."""
    if correction_mode == "disabled":
        return PathCleanupResult(
            poses=list(poses),
            correction_mode=correction_mode,
            input_points=len(poses),
            output_points=len(poses),
            open_space_points=0,
            corridor_points=0,
            cornering_points=0,
            corner_mask_count=0,
            preserved_safe_points=0,
            preserved_outside_corner_points=0,
            repaired_unsafe_points=0,
            corner_adjusted_points=0,
            adjusted_inside_corner_points=0,
            adjusted_points=0,
            unresolved_inside_corner_points=0,
            unresolved_points=0,
            unsafe_outside_corner_ignored_points=0,
            adjusted_indices=[],
        )

    if pre_simplify_enabled:
        poses = simplify_replay_path_points(
            poses=poses,
            min_spacing_m=pre_simplify_min_spacing_m,
            heading_threshold_rad=pre_simplify_heading_threshold_rad,
        )

    if len(poses) < 3:
        return PathCleanupResult(
            poses=list(poses),
            correction_mode=correction_mode,
            input_points=len(poses),
            output_points=len(poses),
            open_space_points=0,
            corridor_points=0,
            cornering_points=0,
            corner_mask_count=0,
            preserved_safe_points=0,
            preserved_outside_corner_points=0,
            repaired_unsafe_points=0,
            corner_adjusted_points=0,
            adjusted_inside_corner_points=0,
            adjusted_points=0,
            unresolved_inside_corner_points=0,
            unresolved_points=0,
            unsafe_outside_corner_ignored_points=0,
            adjusted_indices=[],
        )

    adjusted_poses = list(poses)
    adjusted_points = 0
    unresolved_points = 0
    preserved_safe_points = 0
    repaired_unsafe_points = 0
    corner_adjusted_points = 0
    adjusted_inside_corner_points = 0
    unresolved_inside_corner_points = 0
    unsafe_outside_corner_ignored_points = 0
    preserved_outside_corner_points = 0
    open_space_points = 0
    corridor_points = 0
    cornering_points = 0
    adjusted_indices: list[int] = []
    cumulative_distances = build_cumulative_distances(poses)
    total_distance_m = cumulative_distances[-1]
    path_contexts: list[PathContext | None] = [None] * len(poses)

    for index in range(1, len(poses) - 1):
        distance_from_start_m = cumulative_distances[index]
        distance_to_goal_m = total_distance_m - distance_from_start_m
        if distance_from_start_m < preserve_start_distance_m:
            continue
        if distance_to_goal_m < preserve_goal_distance_m:
            continue

        path_context = classify_path_context(
            previous_pose=adjusted_poses[index - 1],
            current_pose=adjusted_poses[index],
            next_pose=adjusted_poses[index + 1],
            grid=grid,
            clearance_m=clearance_m,
            corridor_max_width_m=corridor_max_width_m,
            corner_heading_threshold_rad=corner_heading_threshold_rad,
            max_lateral_shift_m=max_lateral_shift_m,
        )
        path_contexts[index] = path_context
        if path_context.context_name == "open_space":
            open_space_points += 1
        elif path_context.context_name == "corridor":
            corridor_points += 1
        elif path_context.context_name == "cornering_to_corridor":
            cornering_points += 1

    corner_mask = build_corner_mask(
        path_contexts=path_contexts,
        original_poses=poses,
        adjusted_poses=poses,
        lateral_shifts_m=None,
        corner_heading_threshold_rad=corner_heading_threshold_rad,
        corner_clearance_delta_threshold_m=corner_clearance_delta_threshold_m,
        corner_shift_delta_threshold_m=corner_shift_delta_threshold_m,
        padding_points=corner_segment_padding_points,
    )
    corner_mask_count = sum(1 for is_corner in corner_mask if is_corner)

    for index in range(1, len(poses) - 1):
        distance_from_start_m = cumulative_distances[index]
        distance_to_goal_m = total_distance_m - distance_from_start_m
        if distance_from_start_m < preserve_start_distance_m:
            continue
        if distance_to_goal_m < preserve_goal_distance_m:
            continue

        path_context = path_contexts[index]
        if path_context is None:
            continue

        current_pose = adjusted_poses[index]
        current_pose_is_safe = is_pose_safe(current_pose, grid, clearance_m)
        editable_in_corner_only = bool(corner_mask[index])

        if correction_mode == "corner_only" and not editable_in_corner_only:
            preserved_outside_corner_points += 1
            if current_pose_is_safe:
                preserved_safe_points += 1
            else:
                unsafe_outside_corner_ignored_points += 1
            continue

        decision = choose_cleanup_policy_action(
            correction_mode=correction_mode,
            path_context=path_context,
            current_pose_is_safe=current_pose_is_safe,
        )

        if decision == "preserve":
            if current_pose_is_safe:
                preserved_safe_points += 1
            continue

        use_centering_priority = should_use_centering_priority(
            correction_mode=correction_mode,
            path_context=path_context,
            centering_enabled=centering_enabled,
        )

        candidate_pose = find_best_lateral_candidate(
            adjusted_poses[index - 1],
            current_pose,
            adjusted_poses[index + 1],
            grid,
            clearance_m,
            max_lateral_shift_m,
            lateral_sample_step_m,
            path_context,
            use_centering_priority,
            corner_shift_scale,
        )
        if candidate_pose is None:
            if not current_pose_is_safe:
                unresolved_points += 1
                if correction_mode == "corner_only" and editable_in_corner_only:
                    unresolved_inside_corner_points += 1
            continue

        if poses_are_nearly_equal(current_pose, candidate_pose):
            if current_pose_is_safe:
                preserved_safe_points += 1
            continue

        adjusted_poses[index] = candidate_pose
        adjusted_points += 1
        adjusted_indices.append(index)
        if not current_pose_is_safe:
            repaired_unsafe_points += 1
        if path_context.context_name == "cornering_to_corridor":
            corner_adjusted_points += 1
        if correction_mode == "corner_only" and editable_in_corner_only:
            adjusted_inside_corner_points += 1

    adjusted_poses = apply_cleanup_postprocessing(
        original_poses=poses,
        adjusted_poses=adjusted_poses,
        path_contexts=path_contexts,
        correction_mode=correction_mode,
        grid=grid,
        clearance_m=clearance_m,
        max_lateral_shift_m=max_lateral_shift_m,
        corner_heading_threshold_rad=corner_heading_threshold_rad,
        smoothing_window=smoothing_window,
        transition_distance_m=transition_distance_m,
        preserve_start_distance_m=preserve_start_distance_m,
        preserve_goal_distance_m=preserve_goal_distance_m,
        cumulative_distances=cumulative_distances,
        total_distance_m=total_distance_m,
        max_shift_delta_m=max_shift_delta_m,
        shift_smoothing_window=shift_smoothing_window,
        outlier_threshold_m=outlier_threshold_m,
        corner_clearance_delta_threshold_m=corner_clearance_delta_threshold_m,
        corner_shift_delta_threshold_m=corner_shift_delta_threshold_m,
        corner_segment_padding_points=corner_segment_padding_points,
        corner_curve_resample_step_m=corner_curve_resample_step_m,
    )

    return PathCleanupResult(
        poses=adjusted_poses,
        correction_mode=correction_mode,
        input_points=len(poses),
        output_points=len(adjusted_poses),
        open_space_points=open_space_points,
        corridor_points=corridor_points,
        cornering_points=cornering_points,
        corner_mask_count=corner_mask_count,
        preserved_safe_points=preserved_safe_points,
        preserved_outside_corner_points=preserved_outside_corner_points,
        repaired_unsafe_points=repaired_unsafe_points,
        corner_adjusted_points=corner_adjusted_points,
        adjusted_inside_corner_points=adjusted_inside_corner_points,
        adjusted_points=adjusted_points,
        unresolved_inside_corner_points=unresolved_inside_corner_points,
        unresolved_points=unresolved_points,
        unsafe_outside_corner_ignored_points=unsafe_outside_corner_ignored_points,
        adjusted_indices=adjusted_indices,
    )


def apply_cleanup_postprocessing(
    original_poses: list[RecordedPose],
    adjusted_poses: list[RecordedPose],
    path_contexts: list[PathContext | None],
    correction_mode: str,
    grid: OccupancyGridSnapshot,
    clearance_m: float,
    max_lateral_shift_m: float,
    corner_heading_threshold_rad: float,
    smoothing_window: int,
    transition_distance_m: float,
    preserve_start_distance_m: float,
    preserve_goal_distance_m: float,
    cumulative_distances: list[float],
    total_distance_m: float,
    max_shift_delta_m: float,
    shift_smoothing_window: int,
    outlier_threshold_m: float,
    corner_clearance_delta_threshold_m: float,
    corner_shift_delta_threshold_m: float,
    corner_segment_padding_points: int,
    corner_curve_resample_step_m: float,
) -> list[RecordedPose]:
    """Apply smoothing and stabilization passes after candidate selection."""
    if correction_mode == "safety_only":
        return adjusted_poses

    if correction_mode == "corner_only":
        return smooth_corner_segments(
            original_poses=original_poses,
            adjusted_poses=adjusted_poses,
            path_contexts=path_contexts,
            grid=grid,
            clearance_m=clearance_m,
            max_lateral_shift_m=max_lateral_shift_m,
            corner_heading_threshold_rad=corner_heading_threshold_rad,
            corner_clearance_delta_threshold_m=corner_clearance_delta_threshold_m,
            corner_shift_delta_threshold_m=corner_shift_delta_threshold_m,
            corner_segment_padding_points=corner_segment_padding_points,
            corner_curve_resample_step_m=corner_curve_resample_step_m,
        )

    adjusted_poses = smooth_corridor_segments(
        original_poses=original_poses,
        adjusted_poses=adjusted_poses,
        path_contexts=path_contexts,
        grid=grid,
        clearance_m=clearance_m,
        max_lateral_shift_m=max_lateral_shift_m,
        smoothing_window=smoothing_window,
    )
    adjusted_poses = apply_boundary_transition(
        original_poses=original_poses,
        adjusted_poses=adjusted_poses,
        path_contexts=path_contexts,
        grid=grid,
        clearance_m=clearance_m,
        max_lateral_shift_m=max_lateral_shift_m,
        cumulative_distances=cumulative_distances,
        total_distance_m=total_distance_m,
        preserve_start_distance_m=preserve_start_distance_m,
        preserve_goal_distance_m=preserve_goal_distance_m,
        transition_distance_m=transition_distance_m,
    )
    adjusted_poses = stabilize_lateral_shift_profile(
        original_poses=original_poses,
        adjusted_poses=adjusted_poses,
        path_contexts=path_contexts,
        grid=grid,
        clearance_m=clearance_m,
        max_lateral_shift_m=max_lateral_shift_m,
        max_shift_delta_m=max_shift_delta_m,
        shift_smoothing_window=shift_smoothing_window,
        outlier_threshold_m=outlier_threshold_m,
    )
    return smooth_corner_segments(
        original_poses=original_poses,
        adjusted_poses=adjusted_poses,
        path_contexts=path_contexts,
        grid=grid,
        clearance_m=clearance_m,
        max_lateral_shift_m=max_lateral_shift_m,
        corner_heading_threshold_rad=corner_heading_threshold_rad,
        corner_clearance_delta_threshold_m=corner_clearance_delta_threshold_m,
        corner_shift_delta_threshold_m=corner_shift_delta_threshold_m,
        corner_segment_padding_points=corner_segment_padding_points,
        corner_curve_resample_step_m=corner_curve_resample_step_m,
    )


def choose_cleanup_policy_action(
    correction_mode: str,
    path_context: PathContext,
    current_pose_is_safe: bool,
) -> str:
    """Return the cleanup action for one path point under the current policy."""
    if correction_mode == "corridor_centering":
        return "repair_or_center"

    if correction_mode == "safety_only":
        if current_pose_is_safe:
            return "preserve"
        return "repair_or_center"

    if correction_mode == "corner_only":
        if not current_pose_is_safe:
            return "repair_or_center"
        if path_context.context_name == "cornering_to_corridor":
            return "repair_or_center"
        return "preserve"

    if current_pose_is_safe:
        return "preserve"
    return "repair_or_center"


def should_use_centering_priority(
    correction_mode: str,
    path_context: PathContext,
    centering_enabled: bool,
) -> bool:
    """Return whether candidate search should prefer corridor-centering shifts."""
    if not centering_enabled:
        return False

    if correction_mode == "corridor_centering":
        return True

    if correction_mode == "corner_only":
        return path_context.context_name == "cornering_to_corridor"

    return False


def build_cumulative_distances(poses: list[RecordedPose]) -> list[float]:
    """Return cumulative 2D path distances for each pose."""
    distances = [0.0]
    total_distance_m = 0.0
    for previous_pose, current_pose in zip(poses[:-1], poses[1:]):
        total_distance_m += math.hypot(
            current_pose.x - previous_pose.x,
            current_pose.y - previous_pose.y,
        )
        distances.append(total_distance_m)
    return distances


def simplify_replay_path_points(
    poses: list[RecordedPose],
    min_spacing_m: float,
    heading_threshold_rad: float,
) -> list[RecordedPose]:
    """Remove very dense replay points that add little geometric information."""
    if len(poses) < 3 or min_spacing_m <= 0.0:
        return list(poses)

    simplified_poses = [poses[0]]
    for index in range(1, len(poses) - 1):
        previous_pose = simplified_poses[-1]
        current_pose = poses[index]
        next_pose = poses[index + 1]
        previous_distance_m = (
            (current_pose.x - previous_pose.x) ** 2
            + (current_pose.y - previous_pose.y) ** 2
        ) ** 0.5
        next_distance_m = (
            (next_pose.x - current_pose.x) ** 2
            + (next_pose.y - current_pose.y) ** 2
        ) ** 0.5
        heading_change_rad = compute_heading_change(
            previous_pose,
            current_pose,
            next_pose,
        )
        if (
            previous_distance_m < min_spacing_m
            and next_distance_m < min_spacing_m
            and heading_change_rad < heading_threshold_rad
        ):
            continue
        simplified_poses.append(current_pose)

    simplified_poses.append(poses[-1])
    return simplified_poses


def poses_are_nearly_equal(first_pose: RecordedPose, second_pose: RecordedPose) -> bool:
    """Return True when two 2D poses are effectively the same replay point."""
    return (
        math.hypot(first_pose.x - second_pose.x, first_pose.y - second_pose.y) < 1e-4
        and abs(first_pose.yaw - second_pose.yaw) < 1e-4
    )
