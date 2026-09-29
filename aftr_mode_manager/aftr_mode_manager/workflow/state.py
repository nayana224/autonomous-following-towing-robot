# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Shared command locking and mode-transition rules for AFTR workflows."""

import time

from aftr_mode_manager.mode_state import RobotMode


class WorkflowStateMixin:
    """Provide shared command ownership and validated mode transitions."""

    def set_operator_message(self, message, publish=False):
        """Store one operator-facing progress message.

        Args:
            message: Human-readable Korean progress text.
            publish: Whether to publish status immediately after updating.
        """
        self.status.operator_message = str(message)
        if publish:
            self.publish_status()

    def clear_operator_message(self):
        """Clear the transient operator progress message."""
        self.status.operator_message = ""

    def begin_command(self, response, command_name):
        """Acquire the shared workflow command lock."""
        if not self.command_lock.acquire(blocking=False):
            response.success = False
            active = self.active_command_name or self.status.last_command
            response.message = "mode_manager is busy" + (
                f" with {active}" if active else ""
            )
            self.get_logger().warn(response.message)
            self.status.busy = True
            self.publish_status()
            return False

        self.command_active = True
        self.active_command_name = command_name
        self.status.busy = True
        self.status.last_command = command_name
        self.publish_status()
        return True

    def end_command(self):
        """Release the shared workflow command lock and clear busy state."""
        self.command_active = False
        self.active_command_name = ""
        self.status.busy = False
        self.clear_operator_message()
        self.command_lock.release()
        self.publish_status()

    def run_exclusive_command(self, response, command_name, action):
        """Run one workflow command under the shared busy and lock guard."""
        if not self.begin_command(response, command_name):
            return response
        try:
            return action(response)
        finally:
            self.end_command()

    def request_mode(self, response, mode, command_name, force=False):
        """Validate and apply a requested mode transition."""
        self.status.last_command = command_name

        if self.status.mode == mode:
            response.success = True
            response.message = f"already in {mode.value}"
            self.get_logger().info(response.message)
            self.publish_status()
            return response

        if self.status.mode == RobotMode.ERROR and not force:
            response.success = False
            response.message = "mode manager is in ERROR; call clear_error first"
            self.get_logger().warn(response.message)
            self.publish_status()
            return response

        if not force and not self.is_transition_allowed(self.status.mode, mode):
            response.success = False
            response.message = (
                f"transition not allowed: {self.status.mode.value} -> {mode.value}"
            )
            self.get_logger().warn(response.message)
            self.publish_status()
            return response

        return self.set_mode(response, mode, command_name)

    def is_transition_allowed(self, current_mode, next_mode):
        """Return whether a mode transition is allowed by the state table."""
        return next_mode in self.ALLOWED_TRANSITIONS.get(current_mode, set())

    def set_mode(self, response, mode, command_name):
        """Apply a validated mode transition and fill the Trigger response."""
        self.status.busy = True
        self.status.last_command = command_name
        self.status.mode = mode
        self._update_localizing_transition_state(mode)
        self.status.transition_count += 1
        self.status.busy = False
        self.publish_led_mode_event(mode)

        response.success = True
        response.message = f"mode set to {mode.value}"
        self.get_logger().info(response.message)
        self.publish_status()
        return response

    def set_error(self, message):
        """Move the workflow into ``ERROR`` and store the reason string."""
        self.status.mode = RobotMode.ERROR
        self._update_localizing_transition_state(RobotMode.ERROR)
        self.status.busy = False
        self.clear_operator_message()
        self.status.last_error = message
        self.publish_led_mode_event(RobotMode.ERROR)
        self.get_logger().error(message)
        self.publish_status()

    def _update_localizing_transition_state(self, mode):
        """Track when ``LOCALIZING`` starts so long waits can be detected."""
        if not hasattr(self, "localizing_started_at"):
            return
        if mode == RobotMode.LOCALIZING:
            self.localizing_started_at = time.monotonic()
            return
        self.localizing_started_at = None
