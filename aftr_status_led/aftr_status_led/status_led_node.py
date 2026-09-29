# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""ROS 2 node that drives AFTR status LEDs from mode-manager events."""

from __future__ import annotations

import traceback

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class _MockGpioBackend:
    """Small GPIO-compatible backend used on non-Jetson development hosts."""

    BOARD = "BOARD"
    OUT = "OUT"
    HIGH = 1
    LOW = 0

    def __init__(self, logger) -> None:
        self._logger = logger
        self._pin_values = {}

    def setwarnings(self, _enabled: bool) -> None:
        """Ignore warning toggles to match the Jetson.GPIO API."""

    def setmode(self, mode) -> None:
        """Record the requested numbering mode for debug visibility."""
        self._logger.warn(f"using mock GPIO backend ({mode=})")

    def setup(self, pin: int, _direction, initial=None) -> None:
        """Store the initial value without touching real hardware."""
        self._pin_values[int(pin)] = initial

    def output(self, pin: int, value) -> None:
        """Record one output transition for debug visibility."""
        self._pin_values[int(pin)] = value

    def cleanup(self) -> None:
        """Clear cached pin values."""
        self._pin_values.clear()


class StatusLedNode(Node):
    """Subscribe to LED events and drive three GPIO-backed status lights."""

    def __init__(self) -> None:
        """Initialize parameters, GPIO backend, and ROS interfaces."""
        super().__init__("mdbot_status_led")
        self._declare_parameters()
        self._load_parameters()
        self.current_state = "HOME"
        self.latch_until_sec = None
        self.next_state_after_latch = None
        self.last_outputs = {}
        self._backend = self._create_gpio_backend()
        self._initialize_gpio()
        self._create_ros_interfaces()
        self.apply_outputs(red=False, orange=False, green=False)
        self.publish_current_state()
        self.get_logger().info(
            "mdbot_status_led ready "
            f"(enabled={self.enabled}, topic={self.led_event_topic}, "
            f"use_mock_gpio={self.use_mock_gpio})"
        )

    def _declare_parameters(self) -> None:
        """Declare runtime parameters for GPIO wiring and LED behavior."""
        self.declare_parameter("enabled", True)
        self.declare_parameter("use_mock_gpio", False)
        self.declare_parameter("red_pin", 29)
        self.declare_parameter("orange_pin", 31)
        self.declare_parameter("green_pin", 33)
        self.declare_parameter("active_high", True)
        self.declare_parameter("slow_blink_period_sec", 1.5)
        self.declare_parameter("normal_blink_period_sec", 0.6)
        self.declare_parameter("fast_blink_period_sec", 0.25)
        self.declare_parameter("moving_start_hold_sec", 1.5)
        self.declare_parameter("recovering_hold_sec", 1.5)
        self.declare_parameter("arrived_hold_sec", 2.0)
        self.declare_parameter("failed_blink_sec", 5.0)
        self.declare_parameter("led_event_topic", "/mode_manager/led_event")
        self.declare_parameter(
            "status_led_state_topic",
            "/status_led/current_state",
        )

    def _load_parameters(self) -> None:
        """Read declared parameters into typed instance attributes."""
        self.enabled = bool(self.get_parameter("enabled").value)
        self.use_mock_gpio = bool(self.get_parameter("use_mock_gpio").value)
        self.red_pin = int(self.get_parameter("red_pin").value)
        self.orange_pin = int(self.get_parameter("orange_pin").value)
        self.green_pin = int(self.get_parameter("green_pin").value)
        self.active_high = bool(self.get_parameter("active_high").value)
        self.slow_blink_period_sec = max(
            0.1,
            float(self.get_parameter("slow_blink_period_sec").value),
        )
        self.normal_blink_period_sec = max(
            0.1,
            float(self.get_parameter("normal_blink_period_sec").value),
        )
        self.fast_blink_period_sec = max(
            0.1,
            float(self.get_parameter("fast_blink_period_sec").value),
        )
        self.moving_start_hold_sec = max(
            0.1,
            float(self.get_parameter("moving_start_hold_sec").value),
        )
        self.recovering_hold_sec = max(
            0.1,
            float(self.get_parameter("recovering_hold_sec").value),
        )
        self.arrived_hold_sec = max(
            0.1,
            float(self.get_parameter("arrived_hold_sec").value),
        )
        self.failed_blink_sec = max(
            0.1,
            float(self.get_parameter("failed_blink_sec").value),
        )
        self.led_event_topic = str(self.get_parameter("led_event_topic").value)
        self.status_led_state_topic = str(
            self.get_parameter("status_led_state_topic").value
        )

    def _create_gpio_backend(self):
        """Return either Jetson.GPIO or a mock backend."""
        if not self.enabled:
            return _MockGpioBackend(self.get_logger())
        if self.use_mock_gpio:
            return _MockGpioBackend(self.get_logger())
        try:
            import Jetson.GPIO as jetson_gpio

            return jetson_gpio
        except Exception as error:  # pragma: no cover - hardware-specific path
            self.get_logger().warn(
                "failed to import Jetson.GPIO; falling back to mock backend: "
                f"{error}"
            )
            return _MockGpioBackend(self.get_logger())

    def _initialize_gpio(self) -> None:
        """Set up the three output pins."""
        self._backend.setwarnings(False)
        self._backend.setmode(self._backend.BOARD)
        off_value = self._gpio_off_value()
        for pin in self.output_pins():
            self._backend.setup(pin, self._backend.OUT, initial=off_value)
            self.last_outputs[pin] = None

    def _create_ros_interfaces(self) -> None:
        """Create subscriptions, publishers, services, and timers."""
        self.state_pub = self.create_publisher(
            String,
            self.status_led_state_topic,
            10,
        )
        self.event_sub = self.create_subscription(
            String,
            self.led_event_topic,
            self.led_event_callback,
            20,
        )
        self.timer = self.create_timer(0.05, self.update_leds)
        self._register_debug_services()

    def _register_debug_services(self) -> None:
        """Create optional Trigger services for manual bench testing."""
        self.service_handles = []
        for service_name, state_name in (
            ("/status_led/home", "HOME"),
            ("/status_led/search", "SEARCH"),
            ("/status_led/follow", "FOLLOW"),
            ("/status_led/alignment", "ALIGNMENT"),
            ("/status_led/localizing", "LOCALIZING"),
            ("/status_led/autonomous_ready", "AUTONOMOUS_READY"),
            ("/status_led/obstacle", "OBSTACLE"),
            ("/status_led/moving_start", "MOVING_START"),
            ("/status_led/moving", "MOVING"),
            ("/status_led/blocked", "BLOCKED"),
            ("/status_led/recovering", "RECOVERING"),
            ("/status_led/arrived", "ARRIVED"),
            ("/status_led/failed", "FAILED"),
        ):
            service = self.create_service(
                Trigger,
                service_name,
                self._build_state_service_callback(state_name),
            )
            self.service_handles.append(service)

    def _build_state_service_callback(self, state_name: str):
        """Return a Trigger callback that switches to one state."""

        def callback(_request, response):
            """Apply the requested LED state through the service interface."""
            response.success, response.message = self.set_state(
                state_name,
                source="service",
            )
            return response

        return callback

    @staticmethod
    def valid_states() -> set[str]:
        """Return the accepted upper-case LED event names."""
        return {
            "HOME",
            "SEARCH",
            "FOLLOW",
            "ALIGNMENT",
            "LOCALIZING",
            "AUTONOMOUS_READY",
            "OBSTACLE",
            "MOVING_START",
            "MOVING",
            "BLOCKED",
            "RECOVERING",
            "ARRIVED",
            "FAILED",
        }

    def output_pins(self) -> tuple[int, int, int]:
        """Return the three configured output pins in stable order."""
        return (self.red_pin, self.orange_pin, self.green_pin)

    def _gpio_on_value(self):
        """Return the backend-specific value that means LED on."""
        return self._backend.HIGH if self.active_high else self._backend.LOW

    def _gpio_off_value(self):
        """Return the backend-specific value that means LED off."""
        return self._backend.LOW if self.active_high else self._backend.HIGH

    def now_sec(self) -> float:
        """Return the current ROS time in floating-point seconds."""
        return self.get_clock().now().nanoseconds / 1e9

    def led_event_callback(self, msg: String) -> None:
        """Apply one LED event received from the mode manager."""
        self.set_state(msg.data, source="topic")

    def set_state(self, state: str, source: str = "unknown") -> tuple[bool, str]:
        """Validate and apply one LED state change."""
        normalized = str(state).strip().upper()
        if normalized not in self.valid_states():
            message = f"invalid LED state: {normalized}"
            self.get_logger().warn(message)
            return False, message

        now = self.now_sec()
        self.next_state_after_latch = None
        self.latch_until_sec = None

        if normalized == "MOVING_START":
            self.current_state = normalized
            self.latch_until_sec = now + self.moving_start_hold_sec
            self.next_state_after_latch = "MOVING"
        elif normalized == "RECOVERING":
            self.current_state = normalized
            self.latch_until_sec = now + self.recovering_hold_sec
            self.next_state_after_latch = "MOVING"
        elif normalized == "ARRIVED":
            self.current_state = normalized
            self.latch_until_sec = now + self.arrived_hold_sec
            self.next_state_after_latch = "ALIGNMENT"
        else:
            self.current_state = normalized

        self.update_leds()
        self.publish_current_state()
        message = f"LED state changed to {self.current_state}"
        self.get_logger().info(f"[{source}] {message}")
        return True, message

    def publish_current_state(self) -> None:
        """Publish the current LED state for observability."""
        message = String()
        message.data = self.current_state
        self.state_pub.publish(message)

    def update_leds(self) -> None:
        """Refresh the physical outputs based on the current state."""
        now = self.now_sec()
        if self.latch_until_sec is not None and now >= self.latch_until_sec:
            self.current_state = self.next_state_after_latch or "HOME"
            self.next_state_after_latch = None
            self.latch_until_sec = None
            self.publish_current_state()

        state = self.current_state
        self.apply_outputs(*self.output_flags_for_state(state, now))

    def output_flags_for_state(self, state: str, now_sec: float) -> tuple[bool, bool, bool]:
        """Return ``(red, orange, green)`` output flags for one LED state."""
        style = self.state_style(state)
        if style is None:
            return False, False, False

        color, pattern = style
        if pattern == "steady":
            on = True
        elif pattern == "slow":
            on = self.blink_is_on(now_sec, self.slow_blink_period_sec)
        elif pattern == "normal":
            on = self.blink_is_on(now_sec, self.normal_blink_period_sec)
        elif pattern == "fast":
            on = self.blink_is_on(now_sec, self.fast_blink_period_sec)
        else:
            on = False
        return self.flags_for_color(color, on)

    @staticmethod
    def state_style(state: str) -> tuple[str, str] | None:
        """Return the color/pattern pair used for one logical LED state."""
        styles = {
            "HOME": ("green", "slow"),
            "SEARCH": ("orange", "normal"),
            "FOLLOW": ("green", "slow"),
            "ALIGNMENT": ("orange", "steady"),
            "LOCALIZING": ("orange", "slow"),
            "AUTONOMOUS_READY": ("green", "steady"),
            "OBSTACLE": ("red", "steady"),
            "MOVING_START": ("green", "fast"),
            "MOVING": ("green", "steady"),
            "BLOCKED": ("red", "steady"),
            "RECOVERING": ("orange", "fast"),
            "ARRIVED": ("green", "fast"),
            "FAILED": ("red", "fast"),
        }
        return styles.get(state)

    @staticmethod
    def blink_is_on(now_sec: float, period_sec: float) -> bool:
        """Return whether a blink pattern should be on at one moment."""
        return int(now_sec / period_sec) % 2 == 0

    @staticmethod
    def flags_for_color(color: str, on: bool) -> tuple[bool, bool, bool]:
        """Convert one abstract color into ``(red, orange, green)`` flags."""
        if not on:
            return False, False, False
        if color == "red":
            return True, False, False
        if color == "orange":
            return False, True, False
        if color == "green":
            return False, False, True
        return False, False, False

    def apply_outputs(self, red: bool, orange: bool, green: bool) -> None:
        """Drive the three GPIO pins only when an output value changes."""
        on_value = self._gpio_on_value()
        off_value = self._gpio_off_value()
        desired_outputs = {
            self.red_pin: on_value if red else off_value,
            self.orange_pin: on_value if orange else off_value,
            self.green_pin: on_value if green else off_value,
        }
        for pin, value in desired_outputs.items():
            if self.last_outputs.get(pin) == value:
                continue
            self._backend.output(pin, value)
            self.last_outputs[pin] = value

    def all_off(self) -> None:
        """Turn off all LEDs immediately."""
        self.apply_outputs(red=False, orange=False, green=False)

    def destroy_node(self) -> bool:
        """Ensure LEDs turn off and GPIO state is cleaned up."""
        try:
            self.all_off()
            self._backend.cleanup()
        except Exception as error:  # pragma: no cover - hardware-specific path
            self.get_logger().warn(f"GPIO cleanup failed: {error}")
        return super().destroy_node()


def main(args=None) -> None:
    """Run the AFTR status LED node until shutdown."""
    rclpy.init(args=args)
    node = StatusLedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:  # pragma: no cover - top-level safety net
        node.get_logger().error(traceback.format_exc())
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
