# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
"""Helpers for parsing ``/mode_manager/status`` payloads."""

from __future__ import annotations

import json


def parse_status_payload(text: str) -> dict[str, object]:
    """Parse a JSON status string from ``/mode_manager/status``.

    Args:
        text: Raw status text published by ``mode_manager``.

    Returns:
        A normalized dictionary. Invalid or empty payloads return an empty
        dictionary instead of raising an exception.
    """
    text = text.strip()
    if not text:
        return {}

    try:
        data = json.loads(text)
    except json.JSONDecodeError:
        return {}

    return {str(key): value for key, value in data.items()}


def as_bool(value: object) -> bool:
    """Convert a loosely typed value into ``bool``."""
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def parse_allowed_commands(value: object) -> set[str]:
    """Normalize the ``allowed_commands`` field into a command-key set."""
    if isinstance(value, list):
        return {str(item) for item in value}
    if isinstance(value, tuple):
        return {str(item) for item in value}

    text = str(value).strip()
    if not text:
        return set()
    return {part.strip() for part in text.split(",") if part.strip()}
