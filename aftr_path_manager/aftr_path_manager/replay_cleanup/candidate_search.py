# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Candidate generation and scoring for replay-path centering."""

from __future__ import annotations

from dataclasses import dataclass
import math

from aftr_path_manager.path_types import RecordedPose
from aftr_path_manager.replay_cleanup.grid_utils import OccupancyGridSnapshot
from aftr_path_manager.replay_cleanup.grid_utils import is_pose_safe
from aftr_path_manager.replay_cleanup.grid_utils import measure_directional_clearance
from aftr_path_manager.replay_cleanup.path_context import PathContext
from aftr_path_manager.replay_cleanup.path_context import compute_target_centering_shift
from aftr_path_manager.replay_cleanup.path_context import infer_local_path_yaw


@dataclass
class CandidateScore:
    """Scoring data used to choose the best centered replay candidate."""

    minimum_clearance_m: float
    clearance_balance_m: float
    lateral_offset_m: float


def find_best_lateral_candidate(
    previous_pose: RecordedPose,
    current_pose: RecordedPose,
    next_pose: RecordedPose,
    grid: OccupancyGridSnapshot,
    clearance_m: float,
    max_lateral_shift_m: float,
    lateral_sample_step_m: float,
    path_context: PathContext,
    centering_enabled: bool,
    corner_shift_scale: float,
) -> RecordedPose | None:
    """Return the best safe candidate based on corridor centering and clearance."""
    tangent_yaw = infer_local_path_yaw(previous_pose, current_pose, next_pose)
    normal_x = -math.sin(tangent_yaw)
    normal_y = math.cos(tangent_yaw)
    search_limit_m = max(
        float(max_lateral_shift_m) + float(clearance_m),
        float(clearance_m) * 2.0,
        0.50,
    )
    best_candidate_pose: RecordedPose | None = None
    best_candidate_score: CandidateScore | None = None
    search_offsets = build_search_offsets_for_context(
        path_context=path_context,
        centering_enabled=centering_enabled,
        max_lateral_shift_m=max_lateral_shift_m,
        lateral_sample_step_m=lateral_sample_step_m,
        corner_shift_scale=corner_shift_scale,
    )

    for lateral_offset_m in search_offsets:
        candidate_pose = RecordedPose(
            stamp_sec=current_pose.stamp_sec,
            frame_id=current_pose.frame_id,
            x=current_pose.x + normal_x * lateral_offset_m,
            y=current_pose.y + normal_y * lateral_offset_m,
            yaw=current_pose.yaw,
        )
        if not is_pose_safe(candidate_pose, grid, clearance_m):
            continue

        candidate_score = score_candidate_pose(
            candidate_pose=candidate_pose,
            tangent_yaw=tangent_yaw,
            grid=grid,
            max_search_distance_m=search_limit_m,
            lateral_offset_m=abs(lateral_offset_m),
            path_context=path_context,
        )
        if is_better_candidate_score(candidate_score, best_candidate_score):
            best_candidate_pose = candidate_pose
            best_candidate_score = candidate_score

    return best_candidate_pose


def build_lateral_offsets(
    max_lateral_shift_m: float,
    lateral_sample_step_m: float,
) -> list[float]:
    """Return symmetric lateral offsets ordered from smallest to largest shift."""
    if max_lateral_shift_m <= 0.0 or lateral_sample_step_m <= 0.0:
        return [0.0]

    max_step_count = max(1, int(math.floor(max_lateral_shift_m / lateral_sample_step_m)))
    offsets = [0.0]
    for step_index in range(1, max_step_count + 1):
        offset_m = step_index * lateral_sample_step_m
        offsets.append(offset_m)
        offsets.append(-offset_m)
    return offsets


def build_search_offsets_for_context(
    path_context: PathContext,
    centering_enabled: bool,
    max_lateral_shift_m: float,
    lateral_sample_step_m: float,
    corner_shift_scale: float,
) -> list[float]:
    """Return lateral offsets ordered around the preferred corridor-centered shift."""
    base_offsets = build_lateral_offsets(
        max_lateral_shift_m=max_lateral_shift_m,
        lateral_sample_step_m=lateral_sample_step_m,
    )
    if not centering_enabled or path_context.context_name == "open_space":
        return base_offsets

    target_shift_m = compute_target_centering_shift(
        left_clearance_m=path_context.left_clearance_m,
        right_clearance_m=path_context.right_clearance_m,
        max_lateral_shift_m=max_lateral_shift_m,
    )
    if path_context.context_name == "cornering_to_corridor":
        target_shift_m *= max(0.0, min(1.0, corner_shift_scale))

    return sorted(
        base_offsets,
        key=lambda offset_m: (
            abs(offset_m - target_shift_m),
            abs(offset_m),
        ),
    )


def score_candidate_pose(
    candidate_pose: RecordedPose,
    tangent_yaw: float,
    grid: OccupancyGridSnapshot,
    max_search_distance_m: float,
    lateral_offset_m: float,
    path_context: PathContext,
) -> CandidateScore:
    """Score one safe candidate using corridor width and left/right balance."""
    left_clearance_m = measure_directional_clearance(
        pose=candidate_pose,
        yaw=tangent_yaw + math.pi * 0.5,
        grid=grid,
        max_search_distance_m=max_search_distance_m,
    )
    right_clearance_m = measure_directional_clearance(
        pose=candidate_pose,
        yaw=tangent_yaw - math.pi * 0.5,
        grid=grid,
        max_search_distance_m=max_search_distance_m,
    )
    return CandidateScore(
        minimum_clearance_m=score_minimum_clearance(
            left_clearance_m=left_clearance_m,
            right_clearance_m=right_clearance_m,
            path_context=path_context,
        ),
        clearance_balance_m=score_clearance_balance(
            left_clearance_m=left_clearance_m,
            right_clearance_m=right_clearance_m,
            path_context=path_context,
        ),
        lateral_offset_m=lateral_offset_m,
    )


def score_minimum_clearance(
    left_clearance_m: float,
    right_clearance_m: float,
    path_context: PathContext,
) -> float:
    """Return the primary clearance score for one replay candidate."""
    if path_context.context_name == "open_space":
        return min(left_clearance_m, right_clearance_m)
    corridor_center_bias_m = abs(left_clearance_m - right_clearance_m) * 0.5
    return min(left_clearance_m, right_clearance_m) - corridor_center_bias_m


def score_clearance_balance(
    left_clearance_m: float,
    right_clearance_m: float,
    path_context: PathContext,
) -> float:
    """Return the secondary corridor-balance score for one replay candidate."""
    if path_context.context_name == "open_space":
        return abs(left_clearance_m - right_clearance_m)
    return abs(left_clearance_m - right_clearance_m) * 0.5


def is_better_candidate_score(
    candidate_score: CandidateScore,
    best_score: CandidateScore | None,
) -> bool:
    """Return True when one candidate score is better than the current best."""
    if best_score is None:
        return True

    if candidate_score.minimum_clearance_m > best_score.minimum_clearance_m + 0.03:
        return True
    if best_score.minimum_clearance_m > candidate_score.minimum_clearance_m + 0.03:
        return False

    if candidate_score.clearance_balance_m + 0.03 < best_score.clearance_balance_m:
        return True
    if best_score.clearance_balance_m + 0.03 < candidate_score.clearance_balance_m:
        return False

    return candidate_score.lateral_offset_m < best_score.lateral_offset_m
