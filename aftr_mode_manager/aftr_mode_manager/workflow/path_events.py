# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Path-manager status and replay-event helpers."""

import json


def parse_path_status(text: str) -> dict:
    text = text.strip()
    if not text:
        return {}

    try:
        data = json.loads(text)
    except json.JSONDecodeError:
        return {}

    return data if isinstance(data, dict) else {}


def is_blocked_path_event(event) -> bool:
    if not event:
        return False

    return (
        "temporary failure" in event
        or "obstacle" in event
        or "blocked" in event
    )


def is_terminal_path_failure_event(event) -> bool:
    normalized = str(event).strip().lower()
    return (
        normalized == "goal_rejected"
        or normalized.startswith("aborted_status_")
        or normalized.startswith("failed:")
        or (
            normalized.startswith("blocked:")
            and "retry limit" in normalized
        )
    )
