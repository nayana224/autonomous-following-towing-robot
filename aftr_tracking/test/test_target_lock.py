# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Unit tests for conservative worker target association."""

from aftr_tracking.target_lock import RobustTargetLock
from aftr_tracking.target_lock import TargetLockConfig


def test_search_requires_one_continuous_candidate():
    """Lock only after the same spatial candidate persists."""
    tracker = RobustTargetLock(TargetLockConfig(search_confirm_frames=3))

    assert tracker.search([(1.0, 0.0)], 0.0).state == "SEARCH"
    assert tracker.search([(1.03, 0.01)], 0.1).state == "SEARCH"
    result = tracker.search([(1.06, 0.02)], 0.2)

    assert result.state == "LOCKED"
    assert result.position == (1.06, 0.02)
    assert result.target_id == 1


def test_search_restarts_after_candidate_jump():
    """Reject a new person that appears far from the search candidate."""
    tracker = RobustTargetLock(
        TargetLockConfig(
            search_confirm_frames=3,
            search_match_distance_m=0.2,
        )
    )

    tracker.search([(1.0, 0.0)], 0.0)
    tracker.search([(1.05, 0.0)], 0.1)
    result = tracker.search([(1.8, 0.0)], 0.2)

    assert result.state == "SEARCH"
    assert result.position is None
    assert tracker.candidate_frames == 1


def test_search_tolerates_short_detection_gap():
    """Keep initial confirmation progress through brief scan dropouts."""
    tracker = RobustTargetLock(
        TargetLockConfig(
            search_confirm_frames=3,
            search_max_missed_frames=2,
        )
    )

    tracker.search([(1.0, 0.0)], 0.0)
    gap = tracker.search([], 0.1)
    tracker.search([(1.03, 0.01)], 0.2)
    locked = tracker.search([(1.05, 0.02)], 0.3)

    assert gap.state == "SEARCH"
    assert tracker.search_missed_frames == 0
    assert locked.state == "LOCKED"


def test_locked_identity_survives_smooth_motion():
    """Keep the same target identifier during continuous curved motion."""
    tracker = RobustTargetLock(
        TargetLockConfig(
            search_confirm_frames=1,
            match_gate_m=0.7,
        )
    )
    locked = tracker.search([(1.0, 0.0)], 0.0)

    first = tracker.update([(1.04, 0.05)], 0.1)
    second = tracker.update([(1.08, 0.12)], 0.2)

    assert first.state == "LOCKED"
    assert second.state == "LOCKED"
    assert first.target_id == locked.target_id == second.target_id
    assert second.position is not None
    assert second.position[1] > first.position[1]


def test_close_person_uses_short_prediction_then_reacquires():
    """Bridge brief ambiguity and reacquire without changing target ID."""
    tracker = RobustTargetLock(
        TargetLockConfig(
            search_confirm_frames=1,
            match_gate_m=1.0,
            ambiguity_proximity_m=0.5,
            reacquire_confirm_frames=2,
            ambiguous_prediction_hold_frames=2,
        )
    )
    locked = tracker.search([(1.0, 0.0)], 0.0)

    ambiguous = tracker.update([(1.05, 0.02), (1.05, 0.15)], 0.1)
    confirming = tracker.update([(1.10, 0.05)], 0.2)
    reacquired = tracker.update([(1.15, 0.08)], 0.3)

    assert ambiguous.state == "AMBIGUOUS"
    assert ambiguous.position is not None
    assert confirming.state == "AMBIGUOUS"
    assert confirming.position is not None
    assert reacquired.state == "LOCKED"
    assert reacquired.position is not None
    assert reacquired.target_id == locked.target_id


def test_sustained_ambiguity_expires_prediction_window():
    """Stop predicted control if two similarly likely people remain nearby."""
    tracker = RobustTargetLock(
        TargetLockConfig(
            search_confirm_frames=1,
            match_gate_m=1.0,
            ambiguity_score_margin_m=0.2,
            ambiguous_prediction_hold_frames=2,
        )
    )
    tracker.search([(1.0, 0.0)], 0.0)
    candidates = [(1.04, -0.08), (1.04, 0.08)]

    first = tracker.update(candidates, 0.1)
    second = tracker.update(candidates, 0.2)
    stopped = tracker.update(candidates, 0.3)

    assert first.position is not None
    assert second.position is not None
    assert stopped.state == "AMBIGUOUS"
    assert stopped.position is None


def test_missing_target_predicts_briefly_then_becomes_lost():
    """Bridge a short occlusion, then stop and eventually drop the ID."""
    tracker = RobustTargetLock(
        TargetLockConfig(
            search_confirm_frames=1,
            prediction_hold_frames=2,
            max_missed_frames=3,
        )
    )
    tracker.search([(1.0, 0.0)], 0.0)

    first = tracker.update([], 0.1)
    second = tracker.update([], 0.2)
    stopped = tracker.update([], 0.3)
    lost = tracker.update([], 0.4)

    assert first.state == "OCCLUDED"
    assert second.state == "OCCLUDED"
    assert first.position is not None
    assert second.position is not None
    assert stopped.position is None
    assert lost.state == "LOST"
    assert lost.position is None
