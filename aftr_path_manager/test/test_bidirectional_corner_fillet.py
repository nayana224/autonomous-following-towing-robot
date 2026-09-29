# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Tests for direction-symmetric replay corner fillets."""

import math

from aftr_path_manager.bidirectional_corner_fillet import (
    apply_bidirectional_corner_fillets,
)
from aftr_path_manager.path_types import RecordedPose


def _dense_path(vertices, step_m=0.05):
    """Build evenly spaced map-frame poses through polyline vertices."""
    poses = []
    for start, end in zip(vertices, vertices[1:]):
        length = math.hypot(end[0] - start[0], end[1] - start[1])
        segment_count = max(1, int(round(length / step_m)))
        for index in range(segment_count):
            ratio = index / segment_count
            poses.append(
                RecordedPose(
                    stamp_sec=float(len(poses)),
                    frame_id="map",
                    x=start[0] + (end[0] - start[0]) * ratio,
                    y=start[1] + (end[1] - start[1]) * ratio,
                    yaw=0.0,
                )
            )
    poses.append(
        RecordedPose(
            stamp_sec=float(len(poses)),
            frame_id="map",
            x=vertices[-1][0],
            y=vertices[-1][1],
            yaw=0.0,
        )
    )
    return poses


def _distance_to_right_angle(pose):
    """Return distance to the source polyline with a corner at (5, 0)."""
    horizontal = math.hypot(pose.x - min(max(pose.x, 0.0), 5.0), pose.y)
    vertical = math.hypot(pose.x - 5.0, pose.y - min(max(pose.y, 0.0), 5.0))
    return min(horizontal, vertical)


def _fillet(poses):
    """Apply the operator launch tuning used for cart replay."""
    return apply_bidirectional_corner_fillets(
        poses,
        target_radius_m=2.40,
        heading_threshold_rad=math.radians(25.0),
        max_deviation_m=0.80,
        resample_step_m=0.05,
        minimum_radius_m=0.90,
    )


def test_large_right_angle_uses_curvature_ramp_within_deviation_bound():
    """Meet the requested minimum radius without an abrupt curvature step."""
    source = _dense_path([(0.0, 0.0), (5.0, 0.0), (5.0, 5.0)])

    result = _fillet(source)

    assert result.detected_corners == 1
    assert result.modified_corners == 1
    assert result.minimum_applied_radius_m >= 2.40
    assert result.degraded_corners == 0
    assert result.maximum_curvature_rate_per_m2 <= 0.55
    assert max(_distance_to_right_angle(pose) for pose in result.poses) <= 0.80


def test_reverse_path_produces_exactly_reversed_geometry():
    """Use one geometric curve for normal and reverse replay."""
    source = _dense_path([(0.0, 0.0), (5.0, 0.0), (5.0, 5.0)])

    forward = _fillet(source).poses
    reverse = _fillet(list(reversed(source))).poses

    assert len(forward) == len(reverse)
    for forward_pose, reverse_pose in zip(forward, reversed(reverse)):
        assert math.isclose(forward_pose.x, reverse_pose.x, abs_tol=1e-9)
        assert math.isclose(forward_pose.y, reverse_pose.y, abs_tol=1e-9)


def test_tiny_corner_uses_best_available_smooth_transition():
    """Never fall back to the original instantaneous heading change."""
    source = _dense_path([(0.0, 0.0), (0.6, 0.0), (0.6, 0.6)])

    result = _fillet(source)

    assert len(result.poses) >= 2
    assert result.modified_corners == 1
    assert 0.0 < result.minimum_applied_radius_m < 0.90
    assert result.degraded_corners == 1
    assert math.isclose(result.poses[0].x, source[0].x, abs_tol=1e-9)
    assert math.isclose(result.poses[-1].y, source[-1].y, abs_tol=1e-9)


def test_straight_path_is_unchanged():
    """Avoid modifying normal straight replay segments."""
    source = _dense_path([(0.0, 0.0), (3.0, 0.0)])

    result = _fillet(source)

    assert result.detected_corners == 0
    assert result.modified_corners == 0
    assert result.poses == source


def test_small_recording_heading_jitter_does_not_limit_major_corner():
    """Reserve transition length for the subsequent load-bearing corner."""
    source = _dense_path(
        [
            (0.0, 0.0),
            (5.0, 0.0),
            (6.0, 0.38),
            (6.0, 5.0),
        ]
    )

    result = _fillet(source)

    assert result.detected_corners == 1
    assert result.modified_corners == 1


def test_directional_overshoot_stays_arc_shaped_without_a_loop():
    """Continue the approach outside while progressing along the exit leg."""
    source = _dense_path([(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)])

    result = apply_bidirectional_corner_fillets(
        source,
        target_radius_m=2.40,
        heading_threshold_rad=math.radians(25.0),
        max_deviation_m=0.80,
        resample_step_m=0.05,
        minimum_radius_m=1.50,
        outward_overshoot_m=1.00,
    )

    assert result.modified_corners == 1
    assert max(pose.x for pose in result.poses) > 10.15
    lowest_index = min(
        range(len(result.poses)),
        key=lambda index: result.poses[index].y,
    )
    assert all(
        current.y <= following.y + 1e-9
        for current, following in zip(
            result.poses[lowest_index:],
            result.poses[lowest_index + 1:],
        )
    )
    assert result.minimum_applied_radius_m >= 1.50
    assert result.maximum_curvature_rate_per_m2 <= 5.00
