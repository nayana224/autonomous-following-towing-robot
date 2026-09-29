# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Tests for follow-state LED debounce behavior."""

from types import SimpleNamespace

from aftr_mode_manager.hardened_fall_mode_manager_node import (
    HardenedFallModeManagerNode,
)


def _make_manager():
    """Create a ROS-free callback harness from the concrete manager class."""
    manager = HardenedFallModeManagerNode.__new__(HardenedFallModeManagerNode)
    manager.latest_follow_state = ""
    manager.previous_follow_state = ""
    manager.latest_follow_state_raw = ""
    manager.pending_follow_state = ""
    manager.pending_follow_state_since = None
    manager.follow_obstacle_led_hold_sec = 0.5
    manager.follow_obstacle_clear_hold_sec = 0.4
    manager._update_follow_audio_timestamps = lambda previous, current: None
    manager.publish_follow_audio_transition = lambda previous, current: None
    manager.led_states = []
    manager.publish_led_follow_state = lambda: manager.led_states.append(
        manager.latest_follow_state
    )
    return manager


def test_short_obstacle_does_not_replace_follow_led(monkeypatch):
    """Ignore a red obstacle indication shorter than its hold interval."""
    manager = _make_manager()
    now_sec = [0.0]
    monkeypatch.setattr(
        "aftr_mode_manager.hardened_fall_mode_manager_node.time.monotonic",
        lambda: now_sec[0],
    )

    manager.follow_state_callback(SimpleNamespace(data="FOLLOW"))
    now_sec[0] = 0.1
    manager.follow_state_callback(SimpleNamespace(data="OBSTACLE"))
    now_sec[0] = 0.3
    manager.follow_state_callback(SimpleNamespace(data="FOLLOW"))

    assert manager.latest_follow_state == "FOLLOW"
    assert manager.led_states == ["FOLLOW"]


def test_persistent_obstacle_and_clear_are_confirmed(monkeypatch):
    """Publish red and green only after their respective stable intervals."""
    manager = _make_manager()
    now_sec = [0.0]
    monkeypatch.setattr(
        "aftr_mode_manager.hardened_fall_mode_manager_node.time.monotonic",
        lambda: now_sec[0],
    )

    manager.follow_state_callback(SimpleNamespace(data="FOLLOW"))
    now_sec[0] = 0.1
    manager.follow_state_callback(SimpleNamespace(data="OBSTACLE"))
    now_sec[0] = 0.61
    manager.follow_state_callback(SimpleNamespace(data="OBSTACLE"))
    now_sec[0] = 0.7
    manager.follow_state_callback(SimpleNamespace(data="FOLLOW"))
    now_sec[0] = 1.11
    manager.follow_state_callback(SimpleNamespace(data="FOLLOW"))

    assert manager.latest_follow_state == "FOLLOW"
    assert manager.led_states == ["FOLLOW", "OBSTACLE", "FOLLOW"]
