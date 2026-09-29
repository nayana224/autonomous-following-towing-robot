# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors

"""Audio event policy for AFTR autonomous driving feedback."""

from __future__ import annotations

from dataclasses import dataclass
import time


SFX_PLAYBACK_MODE_QUEUE = "queue"
SFX_PLAYBACK_MODE_INTERRUPT = "interrupt"


@dataclass(frozen=True)
class BgmConfig:
    """Looping background music configuration."""

    file_name: str
    fallback_files: tuple[str, ...]
    loop: bool
    volume: float


@dataclass(frozen=True)
class SfxConfig:
    """One-shot sound-effect configuration."""

    file_name: str
    fallback_files: tuple[str, ...]
    cooldown_sec: float
    volume: float
    stop_bgm_before_play: bool
    playback_mode: str


@dataclass(frozen=True)
class EventAction:
    """One action that the audio node should execute."""

    kind: str
    key: str = ""
    playback_mode: str = ""


@dataclass(frozen=True)
class EventConfig:
    """Event mapping configuration."""

    event_type: str
    key: str = ""


class AudioPolicy:
    """Decide how incoming audio events should be handled."""

    def __init__(
        self,
        bgm_events: dict[str, BgmConfig],
        sfx_events: dict[str, SfxConfig],
        event_map: dict[str, EventConfig],
    ) -> None:
        """Store resolved configuration and runtime cooldown state."""
        self._bgm_events = bgm_events
        self._sfx_events = sfx_events
        self._event_map = event_map
        self._last_sfx_times: dict[str, float] = {}

    @classmethod
    def from_config(
        cls,
        config: dict,
        bgm_volume_scale: float,
        sfx_volume_scale: float,
    ) -> "AudioPolicy":
        """Build the audio policy from a YAML configuration dictionary."""
        bgm_events: dict[str, BgmConfig] = {}
        for key, entry in config.get("bgm", {}).items():
            bgm_events[key] = BgmConfig(
                file_name=entry["file"],
                fallback_files=_read_fallback_files(entry),
                loop=bool(entry.get("loop", True)),
                volume=_clamp_volume(entry.get("volume", 1.0) * bgm_volume_scale),
            )

        sfx_events: dict[str, SfxConfig] = {}
        for key, entry in config.get("sfx", {}).items():
            sfx_events[key] = SfxConfig(
                file_name=entry["file"],
                fallback_files=_read_fallback_files(entry),
                cooldown_sec=max(0.0, float(entry.get("cooldown_sec", 0.0))),
                volume=_clamp_volume(entry.get("volume", 1.0) * sfx_volume_scale),
                stop_bgm_before_play=bool(
                    entry.get("stop_bgm_before_play", False)
                ),
                playback_mode=_read_playback_mode(entry),
            )

        event_map: dict[str, EventConfig] = {}
        for event_name, entry in config.get("events", {}).items():
            event_map[event_name] = EventConfig(
                event_type=str(entry["type"]),
                key=str(entry.get("key", "")),
            )

        return cls(
            bgm_events=bgm_events,
            sfx_events=sfx_events,
            event_map=event_map,
        )

    def build_actions(
        self,
        event_name: str,
        current_bgm_key: str | None,
        logger,
    ) -> list[EventAction]:
        """Return a list of playback actions for one audio event."""
        event_config = self._event_map.get(event_name)
        if event_config is None:
            logger.warn(f"unknown audio event ignored: {event_name}")
            return []

        if event_config.event_type == "bgm_start":
            return self._build_bgm_start_actions(event_config, current_bgm_key)
        if event_config.event_type == "bgm_stop":
            return self._build_bgm_stop_actions(event_config, current_bgm_key)
        if event_config.event_type == "sfx":
            return self._build_sfx_actions(event_config, current_bgm_key)

        logger.warn(
            f"audio event has unsupported type '{event_config.event_type}': "
            f"{event_name}"
        )
        return []

    def get_bgm_config(self, key: str) -> BgmConfig | None:
        """Return BGM configuration by key."""
        return self._bgm_events.get(key)

    def iter_bgm_configs(self):
        """Iterate through configured BGM entries."""
        return self._bgm_events.items()

    def get_sfx_config(self, key: str) -> SfxConfig | None:
        """Return SFX configuration by key."""
        return self._sfx_events.get(key)

    def iter_sfx_configs(self):
        """Iterate through configured SFX entries."""
        return self._sfx_events.items()

    def _build_bgm_start_actions(
        self,
        event_config: EventConfig,
        current_bgm_key: str | None,
    ) -> list[EventAction]:
        """Return actions for a BGM-start event."""
        if not event_config.key:
            return []
        if current_bgm_key == event_config.key:
            return []
        if event_config.key not in self._bgm_events:
            return []
        return [EventAction(kind="start_bgm", key=event_config.key)]

    def _build_bgm_stop_actions(
        self,
        event_config: EventConfig,
        current_bgm_key: str | None,
    ) -> list[EventAction]:
        """Return actions for a BGM-stop event."""
        if current_bgm_key is None:
            return []
        if event_config.key and current_bgm_key != event_config.key:
            return []
        return [EventAction(kind="stop_bgm", key=current_bgm_key)]

    def _build_sfx_actions(
        self,
        event_config: EventConfig,
        current_bgm_key: str | None,
    ) -> list[EventAction]:
        """Return actions for a one-shot sound-effect event."""
        sfx_config = self._sfx_events.get(event_config.key)
        if sfx_config is None:
            return []
        if self._is_sfx_on_cooldown(event_config.key, sfx_config.cooldown_sec):
            return []

        actions: list[EventAction] = []
        if sfx_config.stop_bgm_before_play and current_bgm_key is not None:
            actions.append(EventAction(kind="stop_bgm", key=current_bgm_key))
        actions.append(
            EventAction(
                kind="play_sfx",
                key=event_config.key,
                playback_mode=sfx_config.playback_mode,
            )
        )
        self._last_sfx_times[event_config.key] = time.monotonic()
        return actions

    def _is_sfx_on_cooldown(self, key: str, cooldown_sec: float) -> bool:
        """Return True when the SFX should be skipped due to cooldown."""
        if cooldown_sec <= 0.0:
            return False

        last_time = self._last_sfx_times.get(key)
        if last_time is None:
            return False

        elapsed_sec = time.monotonic() - last_time
        return elapsed_sec < cooldown_sec


def _clamp_volume(volume: float) -> float:
    """Clamp a volume value to pygame's valid 0.0 to 1.0 range."""
    return max(0.0, min(1.0, float(volume)))


def _read_fallback_files(entry: dict) -> tuple[str, ...]:
    """Return configured fallback file names as a normalized tuple."""
    fallback_files = entry.get("fallback_files", [])
    if isinstance(fallback_files, str):
        fallback_files = [fallback_files]
    if not isinstance(fallback_files, list):
        return ()
    return tuple(str(file_name) for file_name in fallback_files if file_name)


def _read_playback_mode(entry: dict) -> str:
    """Return the normalized SFX playback mode."""
    playback_mode = str(entry.get("playback_mode", SFX_PLAYBACK_MODE_QUEUE)).strip()
    if playback_mode == SFX_PLAYBACK_MODE_INTERRUPT:
        return SFX_PLAYBACK_MODE_INTERRUPT
    return SFX_PLAYBACK_MODE_QUEUE
