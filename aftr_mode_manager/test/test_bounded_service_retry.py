# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Tests for bounded, state-aware service retry behavior."""

from aftr_mode_manager.runtime.service_clients import ServiceClientMixin


class _Logger:
    """Minimal logger accepted by the service helper."""

    def warning(self, _message):
        """Ignore warning output during unit tests."""


class _RetryHarness(ServiceClientMixin):
    """Provide deterministic service results without a ROS graph."""

    def __init__(self, results):
        """Store results returned by successive service attempts."""
        self.results = list(results)
        self.calls = 0
        self.last_service_error = ""
        self.logger = _Logger()

    def get_logger(self):
        """Return the minimal test logger."""
        return self.logger

    def call_trigger_service(self, _client, _service_name, _timeout_sec):
        """Return the next predetermined result."""
        self.calls += 1
        result = self.results.pop(0)
        if not result:
            self.last_service_error = "simulated timeout"
        return result


def test_bounded_retry_stops_after_success(monkeypatch):
    """A successful second attempt must prevent a third request."""
    monkeypatch.setattr("time.sleep", lambda _delay: None)
    harness = _RetryHarness([False, True, True])

    assert harness.call_trigger_service_with_retry(
        object(),
        "/test_service",
        timeout_sec=1.0,
        attempts=3,
    )
    assert harness.calls == 2


def test_state_probe_prevents_ambiguous_duplicate(monkeypatch):
    """Visible target state must suppress a retry after a lost response."""
    monkeypatch.setattr("time.sleep", lambda _delay: None)
    harness = _RetryHarness([False, True])

    assert harness.call_trigger_service_with_retry(
        object(),
        "/test_service",
        timeout_sec=1.0,
        attempts=3,
        success_probe=lambda: True,
    )
    assert harness.calls == 1


def test_retry_never_exceeds_attempt_limit(monkeypatch):
    """Persistent failure must stop at the configured bound."""
    monkeypatch.setattr("time.sleep", lambda _delay: None)
    harness = _RetryHarness([False, False, False])

    assert not harness.call_trigger_service_with_retry(
        object(),
        "/test_service",
        timeout_sec=1.0,
        attempts=3,
    )
    assert harness.calls == 3
