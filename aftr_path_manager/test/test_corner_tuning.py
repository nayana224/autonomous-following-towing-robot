# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Tests for parameter-driven replay corner tuning."""

import math

from aftr_path_manager.continuous_clearance_cleanup import (
    _build_continuous_segments,
)
from aftr_path_manager.continuous_clearance_cleanup import _relax_sharp_turns
from aftr_path_manager.endpoint_smoothed_clearance_cleanup import (
    _safe_weighted_smoothing,
)
from aftr_path_manager.endpoint_smoothed_clearance_cleanup import (
    cleanup_path_points,
)
from aftr_path_manager.path_types import RecordedPose
from aftr_path_manager.replay_cleanup.grid_utils import OccupancyGridSnapshot


def _pose(x_m, y_m):
    """Build a map-frame pose for path geometry tests."""
    return RecordedPose(
        stamp_sec=0.0,
        frame_id="map",
        x=float(x_m),
        y=float(y_m),
        yaw=0.0,
    )


def _empty_grid():
    """Build a sufficiently large free occupancy grid."""
    width = 200
    height = 200
    return OccupancyGridSnapshot(
        resolution_m=0.05,
        width=width,
        height=height,
        origin_x_m=-5.0,
        origin_y_m=-5.0,
        cells=[0] * (width * height),
        unknown_is_occupied=True,
    )


def _grid_with_horizontal_wall(wall_y_m):
    """Build a free grid containing one occupied horizontal wall."""
    grid = _empty_grid()
    wall_index = int((wall_y_m - grid.origin_y_m) / grid.resolution_m)
    for x_index in range(grid.width):
        grid.cells[wall_index * grid.width + x_index] = 100
    return grid


def test_corner_smoothing_strength_zero_preserves_saved_corner():
    """Keep the existing path unchanged when smoothing is disabled."""
    poses = [_pose(0.0, 0.0), _pose(1.0, 0.0), _pose(1.0, 1.0)]

    result = _relax_sharp_turns(
        poses,
        poses,
        _empty_grid(),
        clearance_m=0.0,
        heading_threshold_rad=math.radians(15.0),
        smoothing_strength=0.0,
        max_deviation_m=0.20,
        passes=3,
    )

    assert [(pose.x, pose.y) for pose in result] == [
        (pose.x, pose.y) for pose in poses
    ]


def test_corner_smoothing_respects_maximum_saved_path_deviation():
    """Round a corner without moving it beyond the configured bound."""
    poses = [_pose(0.0, 0.0), _pose(1.0, 0.0), _pose(1.0, 1.0)]

    result = _relax_sharp_turns(
        poses,
        poses,
        _empty_grid(),
        clearance_m=0.0,
        heading_threshold_rad=math.radians(15.0),
        smoothing_strength=1.0,
        max_deviation_m=0.20,
        passes=1,
    )

    displacement_m = math.hypot(
        result[1].x - poses[1].x,
        result[1].y - poses[1].y,
    )
    assert 0.19 <= displacement_m <= 0.20 + 1e-9


def test_entry_transition_reaches_farther_than_exit_transition():
    """Apply a longer correction ramp before a corner than after it."""
    poses = [_pose(index * 0.1, 0.0) for index in range(31)]
    adjusted = list(poses)
    adjusted[15] = _pose(1.5, 0.20)

    result = _build_continuous_segments(
        original_poses=poses,
        base_adjusted_poses=adjusted,
        grid=_empty_grid(),
        clearance_m=0.0,
        max_lateral_shift_m=0.50,
        lateral_sample_step_m=0.05,
        entry_transition_distance_m=0.60,
        exit_transition_distance_m=0.25,
    )

    assert result[8].y > 0.0
    assert abs(result[23].y) <= 1e-9


def test_final_smoothing_uses_strength_and_deviation_parameters():
    """Apply the same tuning limits during the final smoothing pass."""
    poses = [_pose(0.0, 0.0), _pose(1.0, 0.0), _pose(1.0, 1.0)]

    disabled = _safe_weighted_smoothing(
        original_poses=poses,
        corrected_poses=poses,
        grid=_empty_grid(),
        clearance_m=0.0,
        passes=3,
        smoothing_strength=0.0,
        max_deviation_m=0.10,
    )
    assert [(pose.x, pose.y) for pose in disabled] == [
        (pose.x, pose.y) for pose in poses
    ]

    smoothed = _safe_weighted_smoothing(
        original_poses=poses,
        corrected_poses=poses,
        grid=_empty_grid(),
        clearance_m=0.0,
        passes=3,
        smoothing_strength=1.0,
        max_deviation_m=0.10,
    )
    displacement_m = math.hypot(
        smoothed[1].x - poses[1].x,
        smoothed[1].y - poses[1].y,
    )
    assert 0.0 < displacement_m <= 0.10 + 1e-9


def test_operator_cleanup_accepts_all_corner_tuning_parameters():
    """Exercise the endpoint-smoothed cleanup used by the operator launch."""
    poses = [
        _pose(0.0, 0.0),
        _pose(0.5, 0.0),
        _pose(1.0, 0.0),
        _pose(1.0, 0.5),
        _pose(1.0, 1.0),
    ]

    result = cleanup_path_points(
        poses=poses,
        grid=_empty_grid(),
        clearance_m=0.0,
        max_lateral_shift_m=0.50,
        lateral_sample_step_m=0.05,
        preserve_start_distance_m=0.0,
        preserve_goal_distance_m=0.0,
        correction_mode="corner_only",
        centering_enabled=True,
        corridor_max_width_m=2.20,
        corner_heading_threshold_rad=math.radians(15.0),
        corner_shift_scale=0.30,
        smoothing_window=3,
        transition_distance_m=0.45,
        max_shift_delta_m=0.04,
        shift_smoothing_window=2,
        outlier_threshold_m=0.08,
        pre_simplify_enabled=True,
        pre_simplify_min_spacing_m=0.10,
        pre_simplify_heading_threshold_rad=math.radians(8.0),
        corner_clearance_delta_threshold_m=0.18,
        corner_shift_delta_threshold_m=0.07,
        corner_segment_padding_points=2,
        corner_curve_resample_step_m=0.06,
        corner_entry_transition_distance_m=0.60,
        corner_exit_transition_distance_m=0.25,
        corner_smoothing_strength=0.65,
        corner_max_deviation_m=0.20,
    )

    assert len(result.poses) == len(poses)


def test_preferred_clearance_moves_long_segment_without_hard_rejection():
    """Gently offset a wall-adjacent segment beyond the hard safety limit."""
    poses = [_pose(-3.0 + index * 0.1, 0.0) for index in range(61)]

    result = cleanup_path_points(
        poses=poses,
        grid=_grid_with_horizontal_wall(-0.65),
        clearance_m=0.60,
        max_lateral_shift_m=0.50,
        lateral_sample_step_m=0.05,
        preserve_start_distance_m=0.25,
        preserve_goal_distance_m=0.25,
        correction_mode="corner_only",
        centering_enabled=True,
        corridor_max_width_m=2.20,
        corner_heading_threshold_rad=math.radians(15.0),
        corner_shift_scale=0.30,
        smoothing_window=3,
        transition_distance_m=0.45,
        max_shift_delta_m=0.03,
        shift_smoothing_window=2,
        outlier_threshold_m=0.08,
        pre_simplify_enabled=True,
        pre_simplify_min_spacing_m=0.10,
        pre_simplify_heading_threshold_rad=math.radians(8.0),
        corner_clearance_delta_threshold_m=0.18,
        corner_shift_delta_threshold_m=0.07,
        corner_segment_padding_points=2,
        corner_curve_resample_step_m=0.06,
        corner_entry_transition_distance_m=1.50,
        corner_exit_transition_distance_m=1.50,
        corner_smoothing_strength=0.70,
        corner_max_deviation_m=0.25,
        preferred_clearance_m=0.75,
        preferred_max_lateral_shift_m=0.35,
    )

    assert len(result.poses) == len(poses)
    assert result.poses[0] == poses[0]
    assert result.poses[-1] == poses[-1]
    assert result.poses[len(result.poses) // 2].y >= 0.20

