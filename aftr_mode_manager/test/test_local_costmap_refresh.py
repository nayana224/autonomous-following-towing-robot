# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Tests for the saved-path local-costmap refresh guard."""

from types import SimpleNamespace

from aftr_mode_manager.workflow.manager import WorkflowManagerMixin


class _Logger:
    """Minimal logger used by the workflow test harness."""

    def info(self, _message):
        """Ignore informational output during unit tests."""


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
