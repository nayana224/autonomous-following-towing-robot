# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Regression tests for the restored field-tested tracker behavior."""

import numpy as np

from aftr_tracking.tracker_node import NearestPersonFollower


class _MessagePublisher:
    """Capture published messages without creating a ROS node."""

    def __init__(self):
        self.messages = []

    def publish(self, message):
        """Store one published message."""
        self.messages.append(message)


def test_follow_state_uses_mode_unless_overridden():
    """Expose stable tracker states to centralized audio and LED handling."""
    follower = NearestPersonFollower.__new__(NearestPersonFollower)
    follower.mode = "SEARCH"
    follower.state_pub = _MessagePublisher()

    follower.publish_follow_state()
    follower.publish_follow_state("OBSTACLE")

    assert [message.data for message in follower.state_pub.messages] == [
        "SEARCH",
        "OBSTACLE",
    ]


def test_missing_detection_retains_last_target_for_grace_period():
    """Preserve the attached tracker's non-interrupting loss behavior."""
    follower = NearestPersonFollower.__new__(NearestPersonFollower)
    follower.locked_target = np.asarray((1.0, 0.1), dtype=np.float32)
    follower.pause_by_obstacle = False
    follower.lost_frames = 0
    follower.max_lost_frames = 16
    follower.candidate_count = 0
    follower.mode = "FOLLOW"

    for _index in range(16):
        target = follower.update_locked_target([])
        assert target is follower.locked_target
        assert follower.mode == "FOLLOW"

    target = follower.update_locked_target([])

    assert target is None
    assert follower.mode == "SEARCH"


def test_search_locks_nearest_candidate_after_required_frames():
    """Restore nearest-worker selection from the attached tracker."""
    follower = NearestPersonFollower.__new__(NearestPersonFollower)
    follower.detection_dist_limit = 2.0
    follower.candidate_count = 19
    follower.required_frames = 20
    follower.lost_frames = 4
    follower.mode = "SEARCH"
    candidates = [
        np.asarray((1.5, 0.0), dtype=np.float32),
        np.asarray((0.8, 0.1), dtype=np.float32),
    ]

    target = follower.search_target(candidates)

    assert target is candidates[1]
    assert follower.mode == "FOLLOW"
    assert follower.lost_frames == 0
