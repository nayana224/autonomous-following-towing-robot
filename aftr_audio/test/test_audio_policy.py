# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors

"""Tests for pure AFTR audio event policy."""

from aftr_audio.audio_policy import AudioPolicy
from aftr_audio.audio_policy import SFX_PLAYBACK_MODE_INTERRUPT


class _Logger:
    def __init__(self):
        self.messages = []

    def warn(self, message):
        self.messages.append(message)


def _policy():
    return AudioPolicy.from_config(
        {
            "bgm": {
                "drive": {
                    "file": "drive.wav",
                    "volume": 0.5,
                }
            },
            "sfx": {
                "arrived": {
                    "file": "arrived.wav",
                    "cooldown_sec": 0.0,
                    "volume": 1.0,
                    "stop_bgm_before_play": True,
                    "playback_mode": "queue",
                },
                "safety": {
                    "file": "safety.wav",
                    "cooldown_sec": 0.0,
                    "volume": 1.0,
                    "stop_bgm_before_play": True,
                    "playback_mode": "interrupt",
                },
            },
            "events": {
                "bgm_start": {"type": "bgm_start", "key": "drive"},
                "bgm_stop": {"type": "bgm_stop", "key": "drive"},
                "arrived": {"type": "sfx", "key": "arrived"},
                "safety": {"type": "sfx", "key": "safety"},
            },
        },
        bgm_volume_scale=1.0,
        sfx_volume_scale=1.0,
    )


def test_bgm_start_and_stop_actions_preserve_current_track_rules():
    """BGM actions should avoid duplicate starts and unrelated stops."""
    policy = _policy()
    logger = _Logger()

    assert [action.kind for action in policy.build_actions(
        "bgm_start",
        current_bgm_key=None,
        logger=logger,
    )] == ["start_bgm"]
    assert policy.build_actions(
        "bgm_start",
        current_bgm_key="drive",
        logger=logger,
    ) == []
    assert policy.build_actions(
        "bgm_stop",
        current_bgm_key="other",
        logger=logger,
    ) == []
    assert [action.kind for action in policy.build_actions(
        "bgm_stop",
        current_bgm_key="drive",
        logger=logger,
    )] == ["stop_bgm"]


def test_sfx_that_stops_bgm_keeps_action_order():
    """SFX requiring BGM stop must stop BGM before playing the cue."""
    policy = _policy()
    logger = _Logger()

    actions = policy.build_actions(
        "arrived",
        current_bgm_key="drive",
        logger=logger,
    )

    assert [action.kind for action in actions] == ["stop_bgm", "play_sfx"]
    assert actions[1].key == "arrived"


def test_interrupt_sfx_keeps_interrupt_playback_mode():
    """Safety cues must retain interrupt semantics."""
    policy = _policy()
    logger = _Logger()

    actions = policy.build_actions(
        "safety",
        current_bgm_key="drive",
        logger=logger,
    )

    assert [action.kind for action in actions] == ["stop_bgm", "play_sfx"]
    assert actions[1].playback_mode == SFX_PLAYBACK_MODE_INTERRUPT


def test_unknown_event_is_ignored_with_warning():
    """Unknown events should remain non-fatal."""
    policy = _policy()
    logger = _Logger()

    assert policy.build_actions(
        "unknown",
        current_bgm_key=None,
        logger=logger,
    ) == []
    assert logger.messages == ["unknown audio event ignored: unknown"]
