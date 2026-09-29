# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
"""Safety-aware status LED node."""

import traceback

import rclpy

from aftr_status_led.status_led_node import StatusLedNode


class SafetyStatusLedNode(StatusLedNode):
    """Add latched fall-safety LED patterns to the existing LED controller."""

    @staticmethod
    def valid_states() -> set[str]:
        """Accept the base LED states and the safety-latch states."""
        return StatusLedNode.valid_states() | {
            "SAFETY_STOP",
            "SAFETY_CLEARING",
            "SAFETY_CLEARED",
        }

    @staticmethod
    def state_style(state: str):
        """Choose the safety pattern before falling back to base styles."""
        safety_styles = {
            "SAFETY_STOP": ("red", "fast"),
            "SAFETY_CLEARING": ("orange", "fast"),
            "SAFETY_CLEARED": ("green", "fast"),
        }
        return safety_styles.get(state, StatusLedNode.state_style(state))

    def set_state(self, state: str, source: str = "unknown"):
        """Keep the safety-stop indication latched until clearing begins."""
        normalized = str(state).strip().upper()
        if self.current_state == "SAFETY_STOP" and normalized not in {
            "SAFETY_STOP",
            "SAFETY_CLEARING",
            "SAFETY_CLEARED",
        }:
            message = f"ignored LED state while safety stop is latched: {normalized}"
            self.get_logger().warning(message)
            return False, message

        success, message = super().set_state(normalized, source=source)
        if not success:
            return success, message

        if normalized == "SAFETY_CLEARED":
            now = self.now_sec()
            self.latch_until_sec = now + self.recovering_hold_sec
            self.next_state_after_latch = "HOME"
        return success, message


def main(args=None):
    """Run the safety-aware LED node and release ROS resources."""
    rclpy.init(args=args)
    node = SafetyStatusLedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        node.get_logger().error(traceback.format_exc())
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
