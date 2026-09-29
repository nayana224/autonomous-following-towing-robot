# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
"""Joystick-to-velocity mapping for manual GUI teleoperation."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class TeleopConfig:
    """Store manual teleoperation scaling and axis behavior.

    Attributes:
        max_linear_mps: Maximum linear command in meters per second.
        max_angular_rps: Maximum angular command in radians per second.
        reverse_joystick: Whether the joystick axes use the reversed field setup.
    """

    max_linear_mps: float = 0.20
    max_angular_rps: float = 0.50
    reverse_joystick: bool = True


def map_joystick_to_cmd(
    x_axis: float,
    y_axis: float,
    config: TeleopConfig,
) -> tuple[float, float]:
    """Convert normalized joystick axes into ``(linear_x, angular_z)``.

    Args:
        x_axis: Normalized horizontal joystick input in ``[-1.0, 1.0]``.
        y_axis: Normalized vertical joystick input in ``[-1.0, 1.0]``.
        config: Teleoperation scaling and axis-direction configuration.

    Returns:
        Tuple of linear x and angular z commands.
    """
    if config.reverse_joystick:
        linear = -float(y_axis) * config.max_linear_mps
        angular_axis = float(x_axis)
        angular = angular_axis * config.max_angular_rps
        return linear, angular

    linear = float(y_axis) * config.max_linear_mps
    angular_axis = float(x_axis) if y_axis > 0.0 else -float(x_axis)
    angular = angular_axis * config.max_angular_rps
    return linear, angular
