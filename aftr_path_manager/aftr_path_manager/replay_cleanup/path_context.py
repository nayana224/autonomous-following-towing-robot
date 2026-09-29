# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Path-context classification helpers for replay cleanup."""

from __future__ import annotations

from dataclasses import dataclass
import math

from aftr_path_manager.path_types import RecordedPose
from aftr_path_manager.replay_cleanup.grid_utils import OccupancyGridSnapshot
from aftr_path_manager.replay_cleanup.grid_utils import measure_directional_clearance


@dataclass
class PathContext:
    """Local environment classification around one replay path point."""

    context_name: str
    left_clearance_m: float
    right_clearance_m: float
    corridor_width_m: float
    heading_change_rad: float


def infer_local_path_yaw(
    previous_pose: RecordedPose,
    current_pose: RecordedPose,
    next_pose: RecordedPose,
) -> float:
    """Infer a local tangent yaw from the neighboring path points."""
    dx = next_pose.x - previous_pose.x
    dy = next_pose.y - previous_pose.y
    if math.hypot(dx, dy) < 1e-6:
        dx = next_pose.x - current_pose.x
        dy = next_pose.y - current_pose.y
    if math.hypot(dx, dy) < 1e-6:
        dx = current_pose.x - previous_pose.x
        dy = current_pose.y - previous_pose.y
    if math.hypot(dx, dy) < 1e-6:
        return current_pose.yaw
    return math.atan2(dy, dx)


def classify_path_context(
    previous_pose: RecordedPose,
    current_pose: RecordedPose,
    next_pose: RecordedPose,
    grid: OccupancyGridSnapshot,
    clearance_m: float,
    corridor_max_width_m: float,
    corner_heading_threshold_rad: float,
    max_lateral_shift_m: float,
) -> PathContext:
    """Classify one path point as open space, corridor, or corridor cornering."""
    tangent_yaw = infer_local_path_yaw(previous_pose, current_pose, next_pose)
    max_search_distance_m = max(
        float(corridor_max_width_m),
        float(max_lateral_shift_m) + float(clearance_m) * 2.0,
        1.0,
    )
    left_clearance_m = measure_directional_clearance(
        pose=current_pose,
        yaw=tangent_yaw + math.pi * 0.5,
        grid=grid,
        max_search_distance_m=max_search_distance_m,
    )
    right_clearance_m = measure_directional_clearance(
        pose=current_pose,
        yaw=tangent_yaw - math.pi * 0.5,
        grid=grid,
        max_search_distance_m=max_search_distance_m,
    )
    corridor_width_m = left_clearance_m + right_clearance_m
    heading_change_rad = compute_heading_change(previous_pose, current_pose, next_pose)

    context_name = "open_space"
    minimum_corridor_side_m = max(clearance_m * 0.8, 0.15)
    if (
        left_clearance_m >= minimum_corridor_side_m
        and right_clearance_m >= minimum_corridor_side_m
        and corridor_width_m <= corridor_max_width_m
    ):
        context_name = "corridor"
        if heading_change_rad >= corner_heading_threshold_rad:
            context_name = "cornering_to_corridor"

    return PathContext(
        context_name=context_name,
        left_clearance_m=left_clearance_m,
        right_clearance_m=right_clearance_m,
        corridor_width_m=corridor_width_m,
        heading_change_rad=heading_change_rad,
    )


def compute_target_centering_shift(
    left_clearance_m: float,
    right_clearance_m: float,
    max_lateral_shift_m: float,
) -> float:
    """Return the lateral shift that moves one point toward the corridor center."""
    centered_shift_m = (left_clearance_m - right_clearance_m) * 0.5
    return clamp(centered_shift_m, -max_lateral_shift_m, max_lateral_shift_m)


def compute_heading_change(
    previous_pose: RecordedPose,
    current_pose: RecordedPose,
    next_pose: RecordedPose,
) -> float:
    """Return the absolute local heading change around one replay path point."""
    incoming_yaw = infer_local_path_yaw(previous_pose, previous_pose, current_pose)
    outgoing_yaw = infer_local_path_yaw(current_pose, next_pose, next_pose)
    return abs(normalize_angle(outgoing_yaw - incoming_yaw))


def normalize_angle(angle_rad: float) -> float:
    """Wrap one angle to the range [-pi, pi]."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def clamp(value: float, minimum_value: float, maximum_value: float) -> float:
    """Clamp one scalar value to the requested range."""
    return max(minimum_value, min(maximum_value, value))
