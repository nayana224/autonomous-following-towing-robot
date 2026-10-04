# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Tests for map freshness and operator recovery guards."""

from types import SimpleNamespace
from unittest.mock import patch

from aftr_mode_manager.fall_aware_mode_manager_node import FallAwareModeManagerNode
from aftr_mode_manager.hardened_fall_mode_manager_node import (
    HardenedFallModeManagerNode,
)
from aftr_mode_manager.mode_manager_node import ModeManagerNode
from aftr_mode_manager.mode_state import RobotMode
from aftr_mode_manager.operator_status import OperatorStatusMixin
from aftr_mode_manager.sequential_fall_mode_manager_node import (
    SequentialFallModeManagerNode,
)
from aftr_mode_manager.validated_map_mode_manager_node import (
    ValidatedMapModeManagerNode,
)


class _Process:
    """Minimal managed-process double with configurable stop behavior."""

    def __init__(self, stop_succeeds=True):
        self.stop_succeeds = stop_succeeds

    def stop(self, _timeout_sec):
        return self.stop_succeeds

    def is_running(self):
        return not self.stop_succeeds


def test_camera_free_diagnostic_mode_does_not_require_detector_heartbeat():
    """Explicitly disabling fall detection must leave mapping commands usable."""
    manager = HardenedFallModeManagerNode.__new__(HardenedFallModeManagerNode)
    manager.fall_detection_required = False
    manager.status = SimpleNamespace(fall_detector_alive=False)
    response = SimpleNamespace(success=False, message="")

    with patch.object(
        FallAwareModeManagerNode,
        "begin_command",
        return_value=True,
    ) as parent_begin:
        assert manager.begin_command(response, "start_recording_follow")

    parent_begin.assert_called_once_with(response, "start_recording_follow")


def test_saved_map_signature_requires_and_detects_a_complete_new_pair(tmp_path):
    """A map save is useful only when YAML and its image both change."""
    map_prefix = tmp_path / "mdbot_map"
    map_yaml = map_prefix.with_suffix(".yaml")
    map_image = map_prefix.with_suffix(".pgm")
    manager = SequentialFallModeManagerNode.__new__(
        SequentialFallModeManagerNode,
    )
    manager.normalized_map_save_file = lambda: str(map_prefix)

    assert manager._saved_map_output_signature() is None

    map_image.write_bytes(b"P5\n1 1\n255\n\x00")
    map_yaml.write_text("image: mdbot_map.pgm\n", encoding="utf-8")
    original = manager._saved_map_output_signature()
    assert original is not None

    map_image.write_bytes(b"P5\n1 1\n255\n\xff")
    updated = manager._saved_map_output_signature()
    assert updated is not None
    assert updated != original


def test_clear_error_stays_in_error_when_a_process_group_survives():
    """Error reset must not claim IDLE while a managed process remains."""
    manager = ModeManagerNode.__new__(ModeManagerNode)
    manager.status = SimpleNamespace(
        mode=RobotMode.ERROR,
        busy=False,
        last_command="",
        last_error="original failure",
        amcl_pose_ready=True,
    )
    manager.shutdown_timeout_sec = 0.1
    manager.follower_node = _Process(stop_succeeds=True)
    manager.nav2_process = _Process(stop_succeeds=False)
    manager.path_process = _Process(stop_succeeds=True)
    manager.slam_process = _Process(stop_succeeds=True)
    manager.pending_drive_direction = ""
    manager.cancel_alignment_confirmation_request = lambda: None
    manager.stop_autonomous_audio = lambda: None
    manager.reset_autonomous_led_state = lambda: None
    manager._cancel_active_path_follow = lambda timeout_sec: None
    manager.log_nav2_stop_request = lambda _reason: None
    manager._set_process_state = lambda *_args: None
    manager.fail_response = lambda response, message: (
        setattr(response, "success", False)
        or setattr(response, "message", message)
        or response
    )
    manager.request_mode = lambda response, mode, command, force=False: (
        setattr(manager.status, "mode", mode)
        or setattr(response, "success", True)
        or response
    )

    response = SimpleNamespace(success=True, message="")
    result = manager.clear_error(response)

    assert not result.success
    assert "Nav2" in result.message
    assert manager.status.mode == RobotMode.ERROR


def test_manual_control_is_enabled_at_home_and_during_active_alignment():
    """The GUI joystick works at home without competing with active workflows."""
    manager = OperatorStatusMixin()
    manager.status = SimpleNamespace(last_command="")
    manager.alignment_control_active = False
    manager.next_drive_direction = "reverse"

    assert manager.operator_view_for_mode(RobotMode.IDLE).manual_control_allowed

    for mode in (
        RobotMode.FOLLOW,
        RobotMode.RECORDING_FOLLOW,
        RobotMode.LOCALIZING,
        RobotMode.AUTONOMOUS_READY,
        RobotMode.AUTONOMOUS_DRIVING,
        RobotMode.ERROR,
    ):
        assert not manager.operator_view_for_mode(mode).manual_control_allowed

    manager.alignment_control_active = True
    assert manager.operator_view_for_mode(
        RobotMode.ALIGNMENT,
    ).manual_control_allowed



def test_recording_start_keeps_runtime_order_before_follower_motion():
    """Recording infrastructure must be ready before follower motion starts."""
    manager = HardenedFallModeManagerNode.__new__(HardenedFallModeManagerNode)
    events = []

    idle_process = SimpleNamespace(is_running=lambda: False)
    manager.slam_process = idle_process
    manager.path_process = idle_process
    manager.follower_node = idle_process
    manager.path_start_record_client = object()
    manager.fresh_map_timeout_sec = 12.0
    manager.latest_path_status = {}
    manager.last_path_follow_event = "idle"
    manager.last_map_message_at = None
    manager.status = SimpleNamespace(mode=RobotMode.IDLE)
    manager.reset_follow_runtime_state = lambda: events.append("reset_follow")
    manager.ensure_base_running = lambda _response: (
        events.append("base") or True
    )
    manager.ensure_slam_running = lambda _response: (
        events.append("slam") or True
    )
    manager._wait_for_fresh_map = lambda _timeout: (
        events.append("fresh_map") or True
    )
    manager.ensure_path_manager_running = lambda _response: (
        events.append("path_manager") or True
    )
    manager.wait_for_path_manager_service = (
        lambda _client, _name, timeout_sec: (
            events.append(("record_service", timeout_sec)) or True
        )
    )
    manager.call_trigger_service = (
        lambda _client, _name, timeout_sec: (
            events.append(("start_record", timeout_sec)) or True
        )
    )
    manager.ensure_follower_node_running = lambda _response: (
        events.append("follower") or True
    )

    def request_mode(response, mode, command_name):
        events.append(("mode", mode, command_name))
        manager.status.mode = mode
        response.success = True
        return response

    manager.request_mode = request_mode
    manager.notify_follow_start = lambda recording: events.append(
        ("notify_follow", recording)
    )

    response = SimpleNamespace(success=False, message="")
    result = manager.start_recording_follow(response)

    assert result.success
    assert events == [
        "reset_follow",
        "base",
        "slam",
        "fresh_map",
        "path_manager",
        ("record_service", 5.0),
        ("start_record", 2.0),
        "follower",
        ("mode", RobotMode.RECORDING_FOLLOW, "start_recording_follow"),
        ("notify_follow", True),
    ]



def test_false_fall_observation_does_not_release_hardened_safety_latch():
    """Detector False must not clear the operator-controlled safety latch."""
    manager = HardenedFallModeManagerNode.__new__(HardenedFallModeManagerNode)
    manager.status = SimpleNamespace(
        fall_detected=True,
        safety_stop_active=True,
    )
    published = []
    manager.publish_status = lambda: published.append(True)

    manager.fall_detected_callback(SimpleNamespace(data=False))

    assert manager.status.fall_detected is False
    assert manager.status.safety_stop_active is True
    assert published == [True]


def test_hardened_safety_stop_uses_base_workflow_stop(monkeypatch):
    """Safety cleanup must keep the existing base stop-workflow dispatch."""
    manager = HardenedFallModeManagerNode.__new__(HardenedFallModeManagerNode)
    response = SimpleNamespace(success=False, message="")
    calls = []

    def base_stop(_self, result):
        calls.append("base_stop")
        result.success = True
        return result

    monkeypatch.setattr(ModeManagerNode, "stop_workflow", base_stop)

    result = manager._stop_workflow_for_safety(response)

    assert result.success
    assert calls == ["base_stop"]



class _Nav2Process:
    """Minimal Nav2 process double for map-reuse tests."""

    def __init__(self, running=True, stop_succeeds=True):
        self.running = running
        self.stop_succeeds = stop_succeeds
        self.stop_calls = []

    def is_running(self):
        return self.running

    def stop(self, timeout_sec):
        self.stop_calls.append(timeout_sec)
        if self.stop_succeeds:
            self.running = False
        return self.stop_succeeds

    def describe(self):
        return "nav2-test-process"


def test_validated_localization_rejects_invalid_map_before_parent_start(monkeypatch):
    """Invalid saved maps must stop before sequential localization begins."""
    manager = ValidatedMapModeManagerNode.__new__(ValidatedMapModeManagerNode)
    manager._saved_map_validation_error = lambda: "saved map missing"
    parent_calls = []

    def parent_start(_self, response):
        parent_calls.append(True)
        response.success = True
        return response

    monkeypatch.setattr(
        SequentialFallModeManagerNode,
        "start_localizing",
        parent_start,
    )
    manager.fail_response = lambda response, message: (
        setattr(response, "success", False)
        or setattr(response, "message", message)
        or response
    )

    response = SimpleNamespace(success=True, message="")
    result = manager.start_localizing(response)

    assert not result.success
    assert "saved map missing" in result.message
    assert parent_calls == []


def test_validated_localization_reuses_nav2_for_same_map(monkeypatch):
    """The same saved-map signature must reuse a running Nav2 instance."""
    manager = ValidatedMapModeManagerNode.__new__(ValidatedMapModeManagerNode)
    signature = ("map.yaml", 1, 2, "map.pgm", 3, 4)
    manager._saved_map_validation_error = lambda: ""
    manager._saved_map_signature = lambda: signature
    manager.nav2_loaded_map_signature = signature
    manager.nav2_process = _Nav2Process(running=True)
    manager.get_logger = lambda: SimpleNamespace(info=lambda _message: None)
    parent_calls = []

    def parent_start(_self, response):
        parent_calls.append("parent")
        response.success = True
        return response

    monkeypatch.setattr(
        SequentialFallModeManagerNode,
        "start_localizing",
        parent_start,
    )

    response = SimpleNamespace(success=False, message="")
    result = manager.start_localizing(response)

    assert result.success
    assert parent_calls == ["parent"]
    assert manager.nav2_process.stop_calls == []
    assert manager.nav2_loaded_map_signature == signature


def test_changed_saved_map_stops_nav2_before_localization(monkeypatch):
    """A changed map must stop the old Nav2 process before parent startup."""
    manager = ValidatedMapModeManagerNode.__new__(ValidatedMapModeManagerNode)
    old_signature = ("old.yaml", 1, 2, "old.pgm", 3, 4)
    new_signature = ("new.yaml", 5, 6, "new.pgm", 7, 8)
    manager._saved_map_validation_error = lambda: ""
    manager._saved_map_signature = lambda: new_signature
    manager.nav2_loaded_map_signature = old_signature
    manager.nav2_process = _Nav2Process(running=True)
    manager.shutdown_timeout_sec = 1.5
    manager.heavy_process_settle_sec = 0.2
    manager.status = SimpleNamespace(
        nav2_running=True,
        nav2_ready=True,
        amcl_pose_ready=True,
    )
    events = []
    manager.log_nav2_stop_request = lambda reason: events.append(("stop", reason))
    manager.get_logger = lambda: SimpleNamespace(info=lambda _message: None)
    manager.fail_response = lambda response, message: (
        setattr(response, "success", False)
        or setattr(response, "message", message)
        or response
    )

    def parent_start(_self, response):
        events.append("parent")
        response.success = True
        return response

    monkeypatch.setattr(
        SequentialFallModeManagerNode,
        "start_localizing",
        parent_start,
    )
    monkeypatch.setattr(
        "aftr_mode_manager.validated_map_mode_manager_node.time.sleep",
        lambda _seconds: events.append("settle"),
    )

    response = SimpleNamespace(success=False, message="")
    result = manager.start_localizing(response)

    assert result.success
    assert events == [
        ("stop", "saved map changed before localization"),
        "settle",
        "parent",
    ]
    assert manager.nav2_process.stop_calls == [1.5]
    assert manager.nav2_loaded_map_signature == new_signature
