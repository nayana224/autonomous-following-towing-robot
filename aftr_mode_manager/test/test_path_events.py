# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Tests for path-manager status and replay-event helpers."""

from aftr_mode_manager.workflow.path_events import is_blocked_path_event
from aftr_mode_manager.workflow.path_events import is_terminal_path_failure_event
from aftr_mode_manager.workflow.path_events import parse_path_status


def test_parse_path_status_accepts_only_json_objects():
    assert parse_path_status("") == {}
    assert parse_path_status("not-json") == {}
    assert parse_path_status("[]") == {}
    assert parse_path_status('{"recording": true}') == {"recording": True}


def test_blocked_path_event_matches_existing_markers():
    assert is_blocked_path_event("temporary failure")
    assert is_blocked_path_event("obstacle detected")
    assert is_blocked_path_event("blocked by costmap")
    assert not is_blocked_path_event("completed")


def test_terminal_path_failure_event_matches_existing_markers():
    assert is_terminal_path_failure_event("goal_rejected")
    assert is_terminal_path_failure_event("aborted_status_6")
    assert is_terminal_path_failure_event("failed: controller error")
    assert is_terminal_path_failure_event("blocked: retry limit reached")
    assert not is_terminal_path_failure_event("blocked: retrying")
    assert not is_terminal_path_failure_event("completed")
