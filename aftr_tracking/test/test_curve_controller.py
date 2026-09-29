# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Regression tests for low-speed worker-following turns."""

from aftr_tracking.curve_tracker_node import CurvedMotionPersonFollower


def test_angular_command_remains_active_at_zero_linear_speed():
    """Do not cancel a worker-facing turn when forward motion is stopped."""
    follower = CurvedMotionPersonFollower.__new__(CurvedMotionPersonFollower)
    follower.last_cmd_v = 0.0
    follower.last_cmd_w = 0.0
    follower.a_lin_follow = 0.8
    follower.a_stop = 1.0
    follower.a_ang_follow = 2.0
    follower.a_ang_stop = 1.0
    follower.v_max_follow = 3.0
    follower.w_max_follow = 1.5
    follower.tracking_linear_scale = 1.0

    linear, angular = follower.apply_slew(0.0, 0.5, 0.1)

    assert linear == 0.0
    assert angular > 0.0


def test_single_leg_fallback_is_follow_only():
    """Use one leg as temporary evidence only after a worker is locked."""
    follower = CurvedMotionPersonFollower.__new__(CurvedMotionPersonFollower)
    follower.single_leg_fallback_enabled = True
    follower.merged_leg_spread_max = 0.35
    follower.detect_leg_candidates = lambda clusters, spread_max=None: [(1.0, 0.0)]
    follower.pair_leg_candidates = lambda candidates: []

    follower.mode = "SEARCH"
    assert follower.detect_humans([]) == []

    follower.mode = "FOLLOW"
    assert follower.detect_humans([]) == [(1.0, 0.0)]
    assert follower.detection_source == "SINGLE_LEG"


def test_uncertain_tracking_slows_linear_but_keeps_turning():
    """Reduce forward motion without suppressing worker-facing rotation."""
    follower = CurvedMotionPersonFollower.__new__(CurvedMotionPersonFollower)
    follower.last_cmd_v = 0.0
    follower.last_cmd_w = 0.0
    follower.a_lin_follow = 10.0
    follower.a_stop = 10.0
    follower.a_ang_follow = 10.0
    follower.a_ang_stop = 10.0
    follower.v_max_follow = 3.0
    follower.w_max_follow = 1.5
    follower.tracking_linear_scale = 0.2

    linear, angular = follower.apply_slew(1.0, 0.5, 0.1)

    assert linear == 0.2
    assert angular == 0.5
