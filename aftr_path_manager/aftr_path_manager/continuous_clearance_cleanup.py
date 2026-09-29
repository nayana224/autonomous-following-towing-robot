# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
"""Segment-oriented expanded-clearance path correction.

This module keeps one lateral correction direction across each obstacle segment.
It replaces point-wise left/right decisions with merged correction runs, smooth
entry and exit transitions, and same-side safety recovery.
"""

from __future__ import annotations

import math

from aftr_path_manager.path_safety_cleanup import cleanup_path_points as base_cleanup_path_points
from aftr_path_manager.path_types import RecordedPose
from aftr_path_manager.replay_cleanup.grid_utils import OccupancyGridSnapshot
from aftr_path_manager.replay_cleanup.grid_utils import is_pose_safe


CHANGE_EPSILON_M = 0.01
MIN_CORRECTION_SEGMENT_M = 0.45
MERGE_GAP_DISTANCE_M = 0.60
MAX_SHIFT_STEP_M = 0.035
SPIKE_DISTANCE_M = 0.07
STRAIGHT_HEADING_THRESHOLD_RAD = math.radians(8.0)


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
    """Repair clearance violations while preserving segment continuity."""
    result = base_cleanup_path_points(
        poses=poses,
        grid=grid,
        clearance_m=clearance_m,
        max_lateral_shift_m=max_lateral_shift_m,
        lateral_sample_step_m=lateral_sample_step_m,
        preserve_start_distance_m=preserve_start_distance_m,
        preserve_goal_distance_m=preserve_goal_distance_m,
        correction_mode="safety_only",
        centering_enabled=False,
        corridor_max_width_m=corridor_max_width_m,
        corner_heading_threshold_rad=corner_heading_threshold_rad,
        corner_shift_scale=corner_shift_scale,
        smoothing_window=smoothing_window,
        transition_distance_m=transition_distance_m,
        max_shift_delta_m=max_shift_delta_m,
        shift_smoothing_window=shift_smoothing_window,
        outlier_threshold_m=outlier_threshold_m,
        pre_simplify_enabled=False,
        pre_simplify_min_spacing_m=pre_simplify_min_spacing_m,
        pre_simplify_heading_threshold_rad=pre_simplify_heading_threshold_rad,
        corner_clearance_delta_threshold_m=corner_clearance_delta_threshold_m,
        corner_shift_delta_threshold_m=corner_shift_delta_threshold_m,
        corner_segment_padding_points=corner_segment_padding_points,
        corner_curve_resample_step_m=corner_curve_resample_step_m,
    )

    preferred_adjusted = result.poses
    preferred_clearance_m = max(float(preferred_clearance_m), clearance_m)
    preferred_shift_m = min(
        max(0.0, float(preferred_max_lateral_shift_m)),
        max_lateral_shift_m,
    )
    if preferred_clearance_m > clearance_m + CHANGE_EPSILON_M:
        preferred_result = base_cleanup_path_points(
            poses=poses,
            grid=grid,
            clearance_m=preferred_clearance_m,
            max_lateral_shift_m=preferred_shift_m,
            lateral_sample_step_m=lateral_sample_step_m,
            preserve_start_distance_m=preserve_start_distance_m,
            preserve_goal_distance_m=preserve_goal_distance_m,
            correction_mode="safety_only",
            centering_enabled=False,
            corridor_max_width_m=corridor_max_width_m,
            corner_heading_threshold_rad=corner_heading_threshold_rad,
            corner_shift_scale=corner_shift_scale,
            smoothing_window=smoothing_window,
            transition_distance_m=transition_distance_m,
            max_shift_delta_m=max_shift_delta_m,
            shift_smoothing_window=shift_smoothing_window,
            outlier_threshold_m=outlier_threshold_m,
            pre_simplify_enabled=False,
            pre_simplify_min_spacing_m=pre_simplify_min_spacing_m,
            pre_simplify_heading_threshold_rad=(
                pre_simplify_heading_threshold_rad
            ),
            corner_clearance_delta_threshold_m=(
                corner_clearance_delta_threshold_m
            ),
            corner_shift_delta_threshold_m=corner_shift_delta_threshold_m,
            corner_segment_padding_points=corner_segment_padding_points,
            corner_curve_resample_step_m=corner_curve_resample_step_m,
        )
        preferred_adjusted = [
            preferred
            if (
                _pose_distance(original, preferred) > CHANGE_EPSILON_M
                and is_pose_safe(preferred, grid, clearance_m)
            )
            else hard
            for original, hard, preferred in zip(
                poses,
                result.poses,
                preferred_result.poses,
            )
        ]

    if len(poses) < 3 or len(result.poses) != len(poses):
        result.correction_mode = "continuous_clearance_base"
        return result

    corrected = _build_continuous_segments(
        original_poses=poses,
        base_adjusted_poses=preferred_adjusted,
        grid=grid,
        clearance_m=clearance_m,
        max_lateral_shift_m=max_lateral_shift_m,
        lateral_sample_step_m=lateral_sample_step_m,
        entry_transition_distance_m=corner_entry_transition_distance_m,
        exit_transition_distance_m=corner_exit_transition_distance_m,
    )
    corrected = _remove_spikes(poses, corrected, grid, clearance_m)
    corrected = _relax_sharp_turns(
        poses,
        corrected,
        grid,
        clearance_m,
        heading_threshold_rad=corner_heading_threshold_rad,
        smoothing_strength=corner_smoothing_strength,
        max_deviation_m=corner_max_deviation_m,
        passes=smoothing_window,
    )
    corrected = _recompute_yaws(corrected)

    result.poses = corrected
    result.output_points = len(corrected)
    result.adjusted_indices = [
        index
        for index, (original, adjusted) in enumerate(zip(poses, corrected))
        if _pose_distance(original, adjusted) > CHANGE_EPSILON_M
    ]
    result.adjusted_points = len(result.adjusted_indices)
    result.correction_mode = "continuous_clearance_segments"
    return result


def _pose_distance(start: RecordedPose, end: RecordedPose) -> float:
    """Return planar displacement between two path poses."""
    return math.hypot(end.x - start.x, end.y - start.y)


def _build_continuous_segments(
    original_poses: list[RecordedPose],
    base_adjusted_poses: list[RecordedPose],
    grid: OccupancyGridSnapshot,
    clearance_m: float,
    max_lateral_shift_m: float,
    lateral_sample_step_m: float,
    entry_transition_distance_m: float,
    exit_transition_distance_m: float,
) -> list[RecordedPose]:
    cumulative = _cumulative_distances(original_poses)
    base_shifts = [
        (adjusted.x - original.x, adjusted.y - original.y)
        for original, adjusted in zip(original_poses, base_adjusted_poses)
    ]
    changed = [math.hypot(dx, dy) > CHANGE_EPSILON_M for dx, dy in base_shifts]
    runs = _merge_runs_by_distance(
        _true_runs(changed),
        cumulative,
        MERGE_GAP_DISTANCE_M,
    )
    if not runs:
        return list(original_poses)

    target_shifts = [(0.0, 0.0) for _ in original_poses]
    side_signs = [0 for _ in original_poses]

    for raw_start, raw_end in runs:
        core_start, core_end = _expand_short_run(
            raw_start,
            raw_end,
            cumulative,
            MIN_CORRECTION_SEGMENT_M,
        )
        transition_start = _index_before_distance(
            cumulative,
            core_start,
            max(0.0, float(entry_transition_distance_m)),
        )
        transition_end = _index_after_distance(
            cumulative,
            core_end,
            max(0.0, float(exit_transition_distance_m)),
        )

        local_normals = [
            _path_normal(original_poses, index)
            for index in range(len(original_poses))
        ]
        signed_samples = []
        for index in range(core_start, core_end + 1):
            normal = local_normals[index]
            projection = _dot(base_shifts[index], normal)
            if abs(projection) > CHANGE_EPSILON_M:
                signed_samples.append(projection)

        if not signed_samples:
            continue

        side_sign = _dominant_sign(signed_samples)
        same_side_magnitudes = sorted(
            abs(value) for value in signed_samples if value * side_sign > 0.0
        )
        if not same_side_magnitudes:
            same_side_magnitudes = sorted(abs(value) for value in signed_samples)

        straight = (
            _segment_heading_change(original_poses, core_start, core_end)
            <= STRAIGHT_HEADING_THRESHOLD_RAD
        )
        segment_magnitude = min(
            _percentile(same_side_magnitudes, 0.75),
            max_lateral_shift_m,
        )

        local_magnitudes = [0.0 for _ in original_poses]
        if straight:
            for index in range(core_start, core_end + 1):
                local_magnitudes[index] = segment_magnitude
        else:
            for index in range(core_start, core_end + 1):
                projection = _dot(base_shifts[index], local_normals[index]) * side_sign
                local_magnitudes[index] = max(0.0, min(projection, max_lateral_shift_m))
            local_magnitudes = _smooth_scalar_range(
                local_magnitudes,
                core_start,
                core_end,
                radius=3,
                fallback=segment_magnitude,
            )

        for index in range(transition_start, transition_end + 1):
            nearest_core = min(max(index, core_start), core_end)
            core_magnitude = local_magnitudes[nearest_core] or segment_magnitude
            blend = _cosine_transition_blend(
                cumulative,
                index,
                core_start,
                core_end,
                transition_start,
                transition_end,
            )
            magnitude = min(core_magnitude * blend, max_lateral_shift_m)
            normal = local_normals[index]
            candidate = (
                normal[0] * side_sign * magnitude,
                normal[1] * side_sign * magnitude,
            )
            _store_compatible_shift(
                target_shifts,
                side_signs,
                index,
                candidate,
                side_sign,
                normal,
            )

    target_shifts = _limit_shift_changes_both_directions(
        target_shifts,
        MAX_SHIFT_STEP_M,
    )

    output: list[RecordedPose] = []
    sample_step = max(0.01, float(lateral_sample_step_m))
    for index, (original, base_adjusted, target_shift) in enumerate(
        zip(original_poses, base_adjusted_poses, target_shifts)
    ):
        candidate = _shift_pose(original, target_shift)
        if is_pose_safe(candidate, grid, clearance_m):
            output.append(candidate)
            continue

        if is_pose_safe(original, grid, clearance_m):
            output.append(original)
            continue

        side_sign = side_signs[index]
        normal = _path_normal(original_poses, index)
        same_side_candidate = _find_safe_same_side_pose(
            pose=original,
            normal=normal,
            side_sign=side_sign,
            initial_magnitude=math.hypot(*target_shift),
            maximum_magnitude=max_lateral_shift_m,
            sample_step_m=sample_step,
            grid=grid,
            clearance_m=clearance_m,
        )
        if same_side_candidate is not None:
            output.append(same_side_candidate)
            continue

        base_shift = (
            base_adjusted.x - original.x,
            base_adjusted.y - original.y,
        )
        base_projection = _dot(base_shift, normal)
        if (
            is_pose_safe(base_adjusted, grid, clearance_m)
            and (side_sign == 0 or base_projection * side_sign >= 0.0)
        ):
            output.append(base_adjusted)
        else:
            output.append(
                base_adjusted
                if is_pose_safe(base_adjusted, grid, clearance_m)
                else original
            )

    return output


def _store_compatible_shift(
    shifts: list[tuple[float, float]],
    side_signs: list[int],
    index: int,
    candidate: tuple[float, float],
    side_sign: int,
    normal: tuple[float, float],
) -> None:
    current = shifts[index]
    if math.hypot(*current) <= CHANGE_EPSILON_M:
        shifts[index] = candidate
        side_signs[index] = side_sign
        return

    current_projection = _dot(current, normal)
    candidate_projection = _dot(candidate, normal)
    if current_projection * candidate_projection >= 0.0:
        if abs(candidate_projection) > abs(current_projection):
            shifts[index] = candidate
            side_signs[index] = side_sign
        return

    if math.hypot(*candidate) > math.hypot(*current):
        shifts[index] = candidate
        side_signs[index] = side_sign


def _find_safe_same_side_pose(
    pose: RecordedPose,
    normal: tuple[float, float],
    side_sign: int,
    initial_magnitude: float,
    maximum_magnitude: float,
    sample_step_m: float,
    grid: OccupancyGridSnapshot,
    clearance_m: float,
) -> RecordedPose | None:
    if side_sign == 0:
        return None

    magnitude = max(sample_step_m, initial_magnitude)
    while magnitude <= maximum_magnitude + 1e-9:
        shift = (
            normal[0] * side_sign * magnitude,
            normal[1] * side_sign * magnitude,
        )
        candidate = _shift_pose(pose, shift)
        if is_pose_safe(candidate, grid, clearance_m):
            return candidate
        magnitude += sample_step_m
    return None


def _merge_runs_by_distance(
    runs: list[tuple[int, int]],
    cumulative: list[float],
    maximum_gap_m: float,
) -> list[tuple[int, int]]:
    if not runs:
        return []

    merged = [runs[0]]
    for start, end in runs[1:]:
        previous_start, previous_end = merged[-1]
        gap = cumulative[start] - cumulative[previous_end]
        if gap <= maximum_gap_m:
            merged[-1] = (previous_start, end)
        else:
            merged.append((start, end))
    return merged


def _true_runs(flags: list[bool]) -> list[tuple[int, int]]:
    runs: list[tuple[int, int]] = []
    start = None
    for index, flag in enumerate(flags):
        if flag and start is None:
            start = index
        elif not flag and start is not None:
            runs.append((start, index - 1))
            start = None
    if start is not None:
        runs.append((start, len(flags) - 1))
    return runs


def _expand_short_run(
    start: int,
    end: int,
    cumulative: list[float],
    minimum_length_m: float,
) -> tuple[int, int]:
    while cumulative[end] - cumulative[start] < minimum_length_m:
        can_expand_left = start > 1
        can_expand_right = end < len(cumulative) - 2
        if not can_expand_left and not can_expand_right:
            break
        if can_expand_left:
            start -= 1
        if cumulative[end] - cumulative[start] >= minimum_length_m:
            break
        if can_expand_right:
            end += 1
    return start, end


def _index_before_distance(
    cumulative: list[float],
    index: int,
    distance_m: float,
) -> int:
    target = cumulative[index] - distance_m
    while index > 0 and cumulative[index - 1] >= target:
        index -= 1
    return index


def _index_after_distance(
    cumulative: list[float],
    index: int,
    distance_m: float,
) -> int:
    target = cumulative[index] + distance_m
    while index < len(cumulative) - 1 and cumulative[index + 1] <= target:
        index += 1
    return index


def _cosine_transition_blend(
    cumulative: list[float],
    index: int,
    core_start: int,
    core_end: int,
    transition_start: int,
    transition_end: int,
) -> float:
    if core_start <= index <= core_end:
        return 1.0
    if index < core_start:
        denominator = max(cumulative[core_start] - cumulative[transition_start], 1e-6)
        linear = (cumulative[index] - cumulative[transition_start]) / denominator
    else:
        denominator = max(cumulative[transition_end] - cumulative[core_end], 1e-6)
        linear = (cumulative[transition_end] - cumulative[index]) / denominator
    linear = max(0.0, min(1.0, linear))
    return 0.5 - 0.5 * math.cos(math.pi * linear)


def _path_normal(poses: list[RecordedPose], index: int) -> tuple[float, float]:
    if index <= 0:
        start, end = poses[0], poses[1]
    elif index >= len(poses) - 1:
        start, end = poses[-2], poses[-1]
    else:
        start, end = poses[index - 1], poses[index + 1]

    dx = end.x - start.x
    dy = end.y - start.y
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        return -math.sin(poses[index].yaw), math.cos(poses[index].yaw)
    return -dy / length, dx / length


def _segment_heading_change(
    poses: list[RecordedPose],
    start: int,
    end: int,
) -> float:
    headings = []
    for index in range(max(1, start), min(len(poses) - 1, end + 1)):
        previous = poses[index - 1]
        current = poses[index]
        headings.append(math.atan2(current.y - previous.y, current.x - previous.x))
    if len(headings) < 2:
        return 0.0
    return max(
        abs(_normalize_angle(second - first))
        for first, second in zip(headings[:-1], headings[1:])
    )


def _smooth_scalar_range(
    values: list[float],
    start: int,
    end: int,
    radius: int,
    fallback: float,
) -> list[float]:
    output = list(values)
    for index in range(start, end + 1):
        left = max(start, index - radius)
        right = min(end, index + radius)
        nonzero = [value for value in values[left:right + 1] if value > 0.0]
        output[index] = sum(nonzero) / len(nonzero) if nonzero else fallback
    return output


def _limit_shift_changes_both_directions(
    shifts: list[tuple[float, float]],
    maximum_delta_m: float,
) -> list[tuple[float, float]]:
    forward = _limit_adjacent_shift_changes(shifts, maximum_delta_m)
    backward = _limit_adjacent_shift_changes(list(reversed(forward)), maximum_delta_m)
    backward.reverse()
    return backward


def _limit_adjacent_shift_changes(
    shifts: list[tuple[float, float]],
    maximum_delta_m: float,
) -> list[tuple[float, float]]:
    if not shifts:
        return []
    output = [shifts[0]]
    for current in shifts[1:]:
        previous = output[-1]
        delta = (current[0] - previous[0], current[1] - previous[1])
        limited_delta = _limit_vector(delta, maximum_delta_m)
        output.append(
            (previous[0] + limited_delta[0], previous[1] + limited_delta[1])
        )
    return output


def _remove_spikes(
    original_poses: list[RecordedPose],
    adjusted_poses: list[RecordedPose],
    grid: OccupancyGridSnapshot,
    clearance_m: float,
) -> list[RecordedPose]:
    cleaned = list(adjusted_poses)
    for index in range(1, len(cleaned) - 1):
        distance = _point_to_segment_distance(
            cleaned[index],
            cleaned[index - 1],
            cleaned[index + 1],
        )
        if distance <= SPIKE_DISTANCE_M:
            continue

        original = original_poses[index]
        if is_pose_safe(original, grid, clearance_m):
            cleaned[index] = original
            continue

        midpoint = _midpoint_pose(cleaned[index], cleaned[index - 1], cleaned[index + 1])
        if is_pose_safe(midpoint, grid, clearance_m):
            cleaned[index] = midpoint
    return cleaned


def _relax_sharp_turns(
    original_poses: list[RecordedPose],
    adjusted_poses: list[RecordedPose],
    grid: OccupancyGridSnapshot,
    clearance_m: float,
    heading_threshold_rad: float,
    smoothing_strength: float,
    max_deviation_m: float,
    passes: int,
) -> list[RecordedPose]:
    """Round sharp path points without rejecting or relocating the full path."""
    relaxed = list(adjusted_poses)
    strength = max(0.0, min(1.0, float(smoothing_strength)))
    threshold = max(0.0, float(heading_threshold_rad))
    maximum_deviation = max(0.0, float(max_deviation_m))
    for _ in range(max(0, int(passes))):
        next_pass = list(relaxed)
        for index in range(1, len(relaxed) - 1):
            incoming = math.atan2(
                relaxed[index].y - relaxed[index - 1].y,
                relaxed[index].x - relaxed[index - 1].x,
            )
            outgoing = math.atan2(
                relaxed[index + 1].y - relaxed[index].y,
                relaxed[index + 1].x - relaxed[index].x,
            )
            if abs(_normalize_angle(outgoing - incoming)) <= threshold:
                continue

            midpoint = _midpoint_pose(
                relaxed[index],
                relaxed[index - 1],
                relaxed[index + 1],
            )
            candidate = _blend_pose(relaxed[index], midpoint, strength)
            candidate = _limit_pose_deviation(
                original_poses[index],
                candidate,
                maximum_deviation,
            )
            if is_pose_safe(candidate, grid, clearance_m):
                next_pass[index] = candidate
            elif is_pose_safe(original_poses[index], grid, clearance_m):
                next_pass[index] = original_poses[index]
        relaxed = next_pass
    return relaxed


def _blend_pose(
    start: RecordedPose,
    end: RecordedPose,
    fraction: float,
) -> RecordedPose:
    """Blend one path point toward its smoothing candidate."""
    ratio = max(0.0, min(1.0, float(fraction)))
    return RecordedPose(
        stamp_sec=start.stamp_sec,
        frame_id=start.frame_id,
        x=start.x + (end.x - start.x) * ratio,
        y=start.y + (end.y - start.y) * ratio,
        yaw=start.yaw,
    )


def _limit_pose_deviation(
    original: RecordedPose,
    candidate: RecordedPose,
    maximum_deviation_m: float,
) -> RecordedPose:
    """Limit a smoothed point's displacement from the saved path."""
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


def _midpoint_pose(
    reference: RecordedPose,
    previous: RecordedPose,
    following: RecordedPose,
) -> RecordedPose:
    return RecordedPose(
        stamp_sec=reference.stamp_sec,
        frame_id=reference.frame_id,
        x=(previous.x + following.x) * 0.5,
        y=(previous.y + following.y) * 0.5,
        yaw=reference.yaw,
    )


def _recompute_yaws(poses: list[RecordedPose]) -> list[RecordedPose]:
    if len(poses) < 2:
        return list(poses)

    output = []
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


def _cumulative_distances(poses: list[RecordedPose]) -> list[float]:
    distances = [0.0]
    for previous, current in zip(poses[:-1], poses[1:]):
        distances.append(
            distances[-1]
            + math.hypot(current.x - previous.x, current.y - previous.y)
        )
    return distances


def _dominant_sign(values: list[float]) -> int:
    positive_weight = sum(abs(value) for value in values if value > 0.0)
    negative_weight = sum(abs(value) for value in values if value < 0.0)
    return 1 if positive_weight >= negative_weight else -1


def _percentile(values: list[float], fraction: float) -> float:
    if not values:
        return 0.0
    if len(values) == 1:
        return values[0]
    position = max(0.0, min(1.0, fraction)) * (len(values) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return values[lower]
    weight = position - lower
    return values[lower] * (1.0 - weight) + values[upper] * weight


def _shift_pose(pose: RecordedPose, shift: tuple[float, float]) -> RecordedPose:
    return RecordedPose(
        stamp_sec=pose.stamp_sec,
        frame_id=pose.frame_id,
        x=pose.x + shift[0],
        y=pose.y + shift[1],
        yaw=pose.yaw,
    )


def _point_to_segment_distance(
    point: RecordedPose,
    start: RecordedPose,
    end: RecordedPose,
) -> float:
    segment_x = end.x - start.x
    segment_y = end.y - start.y
    segment_length_sq = segment_x * segment_x + segment_y * segment_y
    if segment_length_sq <= 1e-12:
        return math.hypot(point.x - start.x, point.y - start.y)
    projection = (
        (point.x - start.x) * segment_x + (point.y - start.y) * segment_y
    ) / segment_length_sq
    projection = max(0.0, min(1.0, projection))
    nearest_x = start.x + projection * segment_x
    nearest_y = start.y + projection * segment_y
    return math.hypot(point.x - nearest_x, point.y - nearest_y)


def _limit_vector(
    vector: tuple[float, float],
    maximum_length: float,
) -> tuple[float, float]:
    length = math.hypot(*vector)
    if length <= maximum_length or length <= 1e-9:
        return vector
    scale = maximum_length / length
    return vector[0] * scale, vector[1] * scale


def _dot(first: tuple[float, float], second: tuple[float, float]) -> float:
    return first[0] * second[0] + first[1] * second[1]


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))
