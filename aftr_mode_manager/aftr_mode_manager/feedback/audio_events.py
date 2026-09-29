# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Audio event helpers for operator-facing workflow feedback.

This module keeps audio event names and publish helpers in one place so the
workflow code does not scatter raw event strings across many branches.
"""

import time

from std_msgs.msg import String

from aftr_mode_manager.mode_state import RobotMode


AUDIO_EVENT_AUTONOMOUS_START = "autonomous_start"
AUDIO_EVENT_SYSTEM_READY = "system_ready"
AUDIO_EVENT_AUTONOMOUS_READY = "autonomous_ready"
AUDIO_EVENT_AUTONOMOUS_BGM_START = "autonomous_bgm_start"
AUDIO_EVENT_AUTONOMOUS_BGM_STOP = "autonomous_bgm_stop"
AUDIO_EVENT_ARRIVED = "arrived"
AUDIO_EVENT_BLOCKED = "blocked"
AUDIO_EVENT_FAILED = "failed"
AUDIO_EVENT_FOLLOW_START = "follow_start"
AUDIO_EVENT_RECORDING_FOLLOW_START = "recording_follow_start"
AUDIO_EVENT_WORKER_DETECTED = "worker_detected"
AUDIO_EVENT_WORKER_LOST = "worker_lost"
AUDIO_EVENT_ALIGNMENT_REQUEST = "alignment_request"
AUDIO_EVENT_ALIGNMENT_CONFIRMATION_REQUEST = "alignment_confirmation_request"
AUDIO_EVENT_ALIGNMENT_COMPLETED = "alignment_completed"
AUDIO_EVENT_FOLLOW_END = "follow_end"
AUDIO_EVENT_RECORDING_FOLLOW_COMPLETED = "recording_follow_completed"
AUDIO_EVENT_RETURN_TO_HOME = "return_to_home"


class AudioEventMixin:
    """Publish workflow-derived audio events for the AFTR audio node."""

    def publish_audio_event(self, event_name):
        """Publish one audio event if the audio publisher is available."""
        if not hasattr(self, "audio_event_pub"):
            return

        message = String()
        message.data = event_name
        self.audio_event_pub.publish(message)
        self.get_logger().info(f"audio event published: {event_name}")

    def reset_autonomous_audio_state(self):
        """Clear cached autonomous audio gating flags."""
        self.autonomous_blocked_audio_active = False

    def notify_system_ready(self):
        """Play the base-ready cue once when the robot is idle and available."""
        if getattr(self, "system_ready_audio_announced", False):
            return

        self.publish_audio_event(AUDIO_EVENT_SYSTEM_READY)
        self.system_ready_audio_announced = True

    def start_autonomous_audio(self):
        """Play the autonomous start cue and begin background audio."""
        self.reset_autonomous_audio_state()
        self.publish_audio_event(AUDIO_EVENT_AUTONOMOUS_START)
        self.publish_audio_event(AUDIO_EVENT_AUTONOMOUS_BGM_START)

    def notify_autonomous_ready(self):
        """Play the ready cue when autonomous driving becomes available."""
        self.publish_audio_event(AUDIO_EVENT_AUTONOMOUS_READY)

    def notify_alignment_request(self):
        """Play the alignment-request cue when manual alignment is needed."""
        self.publish_audio_event(AUDIO_EVENT_ALIGNMENT_REQUEST)

    def notify_alignment_confirmation_request(self):
        """Play the alignment-confirmation cue after autonomous arrival."""
        self.publish_audio_event(AUDIO_EVENT_ALIGNMENT_CONFIRMATION_REQUEST)

    def notify_alignment_completed(self):
        """Play the alignment-completed cue when alignment succeeds."""
        self.publish_audio_event(AUDIO_EVENT_ALIGNMENT_COMPLETED)

    def schedule_alignment_confirmation_request(self):
        """Schedule a delayed alignment-confirmation prompt after arrival."""
        self.schedule_alignment_confirmation_request_for(
            expected_last_commands=("path_completed_alignment",),
        )

    def schedule_alignment_confirmation_request_for(
        self,
        expected_last_commands: tuple[str, ...],
        delay_sec: float | None = None,
    ):
        """Schedule a delayed alignment-confirmation prompt for one flow."""
        self.cancel_alignment_confirmation_request()
        if delay_sec is None:
            delay_sec = float(getattr(self, "alignment_confirmation_delay_sec", 1.5))
        self.alignment_confirmation_expected_commands = tuple(expected_last_commands)
        self.alignment_confirmation_audio_timer = self.create_timer(
            max(0.0, float(delay_sec)),
            self._run_alignment_confirmation_request_timer,
            callback_group=self.callback_group,
        )

    def cancel_alignment_confirmation_request(self):
        """Cancel the pending alignment-confirmation timer if one exists."""
        timer = getattr(self, "alignment_confirmation_audio_timer", None)
        if timer is None:
            return
        timer.cancel()
        self.destroy_timer(timer)
        self.alignment_confirmation_audio_timer = None
        self.alignment_confirmation_expected_commands = ()

    def _run_alignment_confirmation_request_timer(self):
        """Play the delayed alignment-confirmation cue only if it still applies."""
        self.cancel_alignment_confirmation_request()
        if self.status.mode != RobotMode.ALIGNMENT:
            return
        if self.alignment_control_active:
            return
        expected_commands = getattr(
            self,
            "alignment_confirmation_expected_commands",
            (),
        )
        if expected_commands and self.status.last_command not in expected_commands:
            return
        self.notify_alignment_confirmation_request()

    def stop_autonomous_audio(self):
        """Stop autonomous background audio and clear gating flags."""
        self.publish_audio_event(AUDIO_EVENT_AUTONOMOUS_BGM_STOP)
        self.reset_autonomous_audio_state()

    def notify_autonomous_arrived(self):
        """Play the arrival cue and clear blocked-event gating."""
        self.publish_audio_event(AUDIO_EVENT_ARRIVED)
        self.reset_autonomous_audio_state()

    def notify_autonomous_failed(self):
        """Play the failure cue and clear blocked-event gating."""
        self.publish_audio_event(AUDIO_EVENT_FAILED)
        self.reset_autonomous_audio_state()

    def notify_autonomous_blocked(self):
        """Play the blocked cue once until the blocked state clears."""
        if self.autonomous_blocked_audio_active:
            return

        self.publish_audio_event(AUDIO_EVENT_BLOCKED)
        self.autonomous_blocked_audio_active = True

    def clear_autonomous_blocked(self):
        """Allow the blocked cue to be played again after recovery."""
        self.autonomous_blocked_audio_active = False

    def notify_follow_start(self, recording=False):
        """Play the start cue for follow or recording-follow sessions."""
        event_name = (
            AUDIO_EVENT_RECORDING_FOLLOW_START
            if recording
            else AUDIO_EVENT_FOLLOW_START
        )
        self.publish_audio_event(event_name)

    def publish_follow_audio_transition(self, previous_state, current_state):
        """Publish follow-mode cues only when the follow state truly changes."""
        if self.status.mode not in {RobotMode.FOLLOW, RobotMode.RECORDING_FOLLOW}:
            return

        previous = str(previous_state).strip().upper()
        current = str(current_state).strip().upper()
        if not current or current == previous:
            return

        if current == "FOLLOW" and previous != "FOLLOW":
            if not self.should_announce_worker_detected(previous):
                self.follow_detected_announced = True
                return
            self.publish_audio_event(AUDIO_EVENT_WORKER_DETECTED)
            self.follow_detected_announced = True
            return

        if previous == "FOLLOW" and current == "SEARCH":
            self.publish_audio_event(AUDIO_EVENT_WORKER_LOST)

    def should_announce_worker_detected(self, previous_state):
        """Return whether the worker-detected cue is useful right now."""
        previous = str(previous_state).strip().upper()
        if not self.follow_detected_announced:
            return True
        if previous != "SEARCH":
            return False

        search_started_at = getattr(self, "follow_search_started_at", None)
        if search_started_at is None:
            return False

        min_search_sec = max(
            0.0,
            float(getattr(self, "follow_reacquire_audio_min_search_sec", 1.2)),
        )
        return (time.monotonic() - search_started_at) >= min_search_sec

    def notify_follow_end(self):
        """Play the follow-end cue when a follow session finishes."""
        self.publish_audio_event(AUDIO_EVENT_FOLLOW_END)

    def notify_recording_follow_completed(self):
        """Play the single completion cue for recording-follow shutdown."""
        self.publish_audio_event(AUDIO_EVENT_RECORDING_FOLLOW_COMPLETED)

    def notify_return_to_home(self):
        """Play the shared cue used when the operator returns to home."""
        self.publish_audio_event(AUDIO_EVENT_RETURN_TO_HOME)
