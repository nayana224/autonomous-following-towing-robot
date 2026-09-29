# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Status LED event helpers for operator-facing workflow feedback."""

from std_msgs.msg import String

from aftr_mode_manager.mode_state import RobotMode


LED_EVENT_HOME = "home"
LED_EVENT_SEARCH = "search"
LED_EVENT_FOLLOW = "follow"
LED_EVENT_ALIGNMENT = "alignment"
LED_EVENT_LOCALIZING = "localizing"
LED_EVENT_AUTONOMOUS_READY = "autonomous_ready"
LED_EVENT_OBSTACLE = "obstacle"
LED_EVENT_MOVING_START = "moving_start"
LED_EVENT_MOVING = "moving"
LED_EVENT_BLOCKED = "blocked"
LED_EVENT_RECOVERING = "recovering"
LED_EVENT_ARRIVED = "arrived"
LED_EVENT_FAILED = "failed"


class LedEventMixin:
    """Publish workflow-derived LED events for the AFTR status LED node."""

    def publish_led_event(self, event_name: str) -> None:
        """Publish one LED event if the LED publisher is available."""
        if not hasattr(self, "led_event_pub"):
            return

        message = String()
        message.data = str(event_name)
        self.led_event_pub.publish(message)
        self.get_logger().info(f"LED event published: {event_name}")

    def reset_autonomous_led_state(self) -> None:
        """Clear cached autonomous LED gating flags."""
        self.autonomous_blocked_led_active = False

    def follow_led_event_name(self) -> str:
        """Return the LED event that best matches the cached follow state."""
        state = str(self.latest_follow_state).strip().upper()
        if state == "FOLLOW":
            return LED_EVENT_FOLLOW
        if state == "OBSTACLE":
            return LED_EVENT_OBSTACLE
        return LED_EVENT_SEARCH

    def publish_led_mode_event(self, mode: RobotMode) -> None:
        """Publish the default LED event for one high-level workflow mode."""
        if mode in {RobotMode.FOLLOW, RobotMode.RECORDING_FOLLOW}:
            self.publish_led_event(self.follow_led_event_name())
            return
        if mode == RobotMode.ALIGNMENT:
            self.publish_led_event(LED_EVENT_ALIGNMENT)
            return
        if mode == RobotMode.LOCALIZING:
            self.publish_led_event(LED_EVENT_LOCALIZING)
            return
        if mode == RobotMode.AUTONOMOUS_READY:
            self.publish_led_event(LED_EVENT_AUTONOMOUS_READY)
            return
        if mode == RobotMode.AUTONOMOUS_DRIVING:
            self.publish_led_event(LED_EVENT_MOVING)
            return
        if mode == RobotMode.ERROR:
            self.publish_led_event(LED_EVENT_FAILED)
            return
        self.publish_led_event(LED_EVENT_HOME)

    def publish_led_follow_state(self) -> None:
        """Publish a follow LED event only while a follow mode is active."""
        if self.status.mode not in {RobotMode.FOLLOW, RobotMode.RECORDING_FOLLOW}:
            return
        self.publish_led_event(self.follow_led_event_name())

    def start_autonomous_led(self) -> None:
        """Show the transient autonomous-start indication."""
        self.reset_autonomous_led_state()
        self.publish_led_event(LED_EVENT_MOVING_START)

    def notify_autonomous_arrived_led(self) -> None:
        """Show the autonomous-arrived indication and clear blocked gating."""
        self.publish_led_event(LED_EVENT_ARRIVED)
        self.reset_autonomous_led_state()

    def notify_autonomous_failed_led(self) -> None:
        """Show the failure indication and clear blocked gating."""
        self.publish_led_event(LED_EVENT_FAILED)
        self.reset_autonomous_led_state()

    def notify_autonomous_blocked_led(self) -> None:
        """Show the blocked indication once until recovery happens."""
        if self.autonomous_blocked_led_active:
            return
        self.publish_led_event(LED_EVENT_BLOCKED)
        self.autonomous_blocked_led_active = True

    def clear_autonomous_blocked_led(self) -> None:
        """Show recovery when a previously blocked replay becomes healthy."""
        if not self.autonomous_blocked_led_active:
            return
        self.publish_led_event(LED_EVENT_RECOVERING)
        self.autonomous_blocked_led_active = False
