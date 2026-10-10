# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Tests for the saved-path local-costmap refresh guard."""

from types import SimpleNamespace

from aftr_mode_manager.mode_state import RobotMode
from aftr_mode_manager.workflow.manager import WorkflowManagerMixin


class _Logger:
    """Minimal logger used by the workflow test harness."""

    def info(self, _message):
        """Ignore informational output during unit tests."""

    def warning(self, _message):
        """Ignore warning output during unit tests."""


class _Future:
    """Return a predetermined costmap service result."""

    def __init__(self, result):
        """Store the result returned by ``result``."""
        self._result = result

    def result(self):
        """Return the stored service response."""
        return self._result


class _Client:
    """Minimal ClearEntireCostmap client double."""

    def __init__(self, available=True, result=None):
        """Store service availability and response behavior."""
        self.available = available
        self.response = object() if result is None else result
        self.requests = []

    def wait_for_service(self, timeout_sec):
        """Return the configured availability result."""
        assert timeout_sec == 3.0
        return self.available

    def call_async(self, request):
        """Record the request and return a completed future double."""
        self.requests.append(request)
        return _Future(self.response)


class _ScanReadiness:
    """Return a predetermined fresh-scan result."""

    def __init__(self, fresh):
        """Store whether a fresh scan should be observed."""
        self.fresh = fresh
        self.timeouts = []

    def wait_for_new_message(self, timeout_sec):
        """Record the timeout and return the configured result."""
        self.timeouts.append(timeout_sec)
        return self.fresh


class _RefreshHarness(WorkflowManagerMixin):
    """Provide ROS-free dependencies for the refresh workflow."""

    def __init__(self, available=True, future_done=True, fresh_scan=True):
        """Configure service and scan outcomes for one test."""
        self.local_costmap_clear_client = _Client(available=available)
        self.scan_message_readiness = _ScanReadiness(fresh=fresh_scan)
        self.future_done = future_done
        self.rejection = ""
        self.logger = _Logger()

    def wait_for_future(self, _future, timeout_sec):
        """Return the configured service-future completion result."""
        assert timeout_sec == 3.0
        return self.future_done

    def reject_response(self, response, message):
        """Record the rejection without requiring full mode-manager state."""
        self.rejection = message
        response.success = False
        response.message = message
        return response

    def get_logger(self):
        """Return the minimal logger."""
        return self.logger


def test_costmap_refresh_requires_service_and_fresh_scan():
    """Successful refresh must clear once and observe a later scan."""
    harness = _RefreshHarness()
    response = SimpleNamespace(success=True, message="")

    assert harness.refresh_local_costmap_before_replay(response)
    assert len(harness.local_costmap_clear_client.requests) == 1
    assert harness.scan_message_readiness.timeouts == [2.0]
    assert harness.rejection == ""


def test_costmap_refresh_rejects_missing_fresh_scan():
    """Do not dispatch a path while laser data is stale after clearing."""
    harness = _RefreshHarness(fresh_scan=False)
    response = SimpleNamespace(success=True, message="")

    assert not harness.refresh_local_costmap_before_replay(response)
    assert response.success is False
    assert "fresh /scan" in harness.rejection


def test_costmap_refresh_rejects_unavailable_clear_service():
    """Do not dispatch a path when the Nav2 clear service is unavailable."""
    harness = _RefreshHarness(available=False)
    response = SimpleNamespace(success=True, message="")

    assert not harness.refresh_local_costmap_before_replay(response)
    assert response.success is False
    assert "service unavailable" in harness.rejection


class _ReplayHarness(WorkflowManagerMixin):
    """Provide ROS-free dependencies for saved-path replay tests."""

    def __init__(self, service_succeeds=True, following_path=False):
        self.events = []
        self.next_drive_direction = "reverse"
        self.status = SimpleNamespace(
            mode=RobotMode.AUTONOMOUS_READY,
            last_command="",
            last_error="",
        )
        self.latest_path_status = {"following_path": following_path}
        self.last_path_follow_event = "idle"
        self.last_service_error = "service timeout"
        self.pending_drive_direction = ""
        self.active_drive_direction = ""
        self.path_replay_service_timeout_sec = 120.0
        self.path_follow_saved_path_client = object()
        self.path_follow_saved_path_reverse_client = object()
        self.service_succeeds = service_succeeds
        self.logger = _Logger()

    def ensure_path_manager_running(self, _response):
        self.events.append("path_manager")
        return True

    def _require_amcl_pose_ready(self, _response, _message):
        self.events.append("amcl")
        return True

    def check_nav2_ready(self, response):
        self.events.append("nav2")
        response.success = True
        return response

    def wait_for_stable_autonomous_readiness(self, _response):
        self.events.append("stable_ready")
        return True

    def set_operator_message(self, _message, publish=False):
        self.events.append(("operator_message", publish))

    def refresh_local_costmap_before_replay(self, _response):
        self.events.append("costmap")
        return True

    def call_trigger_service(self, _client, service_name, timeout_sec):
        self.events.append(("path_service", service_name, timeout_sec))
        return self.service_succeeds

    def notify_autonomous_failed(self):
        self.events.append("autonomous_failed")

    def reject_response(self, response, message):
        response.success = False
        response.message = message
        return response

    def request_mode(self, response, mode, command_name):
        self.events.append(("mode", mode, command_name))
        self.status.mode = mode
        response.success = True
        response.message = f"mode set to {mode.value}"
        return response

    def start_autonomous_audio(self):
        self.events.append("audio")

    def start_autonomous_led(self):
        self.events.append("led")

    def ensure_rviz_running_for_autonomous(self):
        self.events.append("rviz")

    def publish_status(self):
        self.events.append("publish_status")

    def get_logger(self):
        return self.logger


def test_saved_path_replay_preserves_readiness_and_dispatch_order():
    """Replay dispatch must happen only after all readiness guards pass."""
    harness = _ReplayHarness()
    response = SimpleNamespace(success=False, message="")

    result = harness.start_saved_path_driving(
        response,
        reverse=True,
        command_name="path_reverse",
    )

    assert result.success
    assert harness.active_drive_direction == "reverse"
    assert harness.pending_drive_direction == ""
    assert harness.events == [
        "path_manager",
        "amcl",
        "nav2",
        "stable_ready",
        ("operator_message", True),
        "costmap",
        (
            "path_service",
            "/path_manager/follow_saved_path_reverse",
            120.0,
        ),
        ("mode", RobotMode.AUTONOMOUS_DRIVING, "path_reverse"),
        "audio",
        "led",
        "rviz",
    ]


def test_saved_path_replay_reconciles_lost_service_response_from_status():
    """An active path-manager status must survive a lost service response."""
    harness = _ReplayHarness(
        service_succeeds=False,
        following_path=True,
    )
    response = SimpleNamespace(success=False, message="")

    result = harness.start_saved_path_driving(
        response,
        reverse=True,
        command_name="path_reverse",
    )

    assert result.success
    assert harness.last_service_error == ""
    assert harness.status.mode == RobotMode.AUTONOMOUS_DRIVING
    assert "autonomous_failed" not in harness.events
