# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors

"""Sound-effect playback helpers for AFTR audio feedback."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import os


@dataclass(frozen=True)
class QueuedSoundEffect:
    """One queued SFX playback request."""

    event_key: str
    file_path: str
    volume: float


class SoundEffectPlayer:
    """Play one-shot sound effects on a mixer channel separate from BGM."""

    def __init__(self, mixer, logger, channel_index: int = 0) -> None:
        """Store the pygame mixer handle and create one dedicated SFX channel."""
        self._mixer = mixer
        self._logger = logger
        self._channel = mixer.Channel(channel_index)
        self._cache: dict[str, object] = {}
        self._queue: deque[QueuedSoundEffect] = deque()

    def enqueue(
        self,
        event_key: str,
        file_path: str,
        volume: float,
        interrupt: bool = False,
    ) -> None:
        """Queue one SFX request or interrupt the current cue when needed."""
        if not os.path.exists(file_path):
            self._logger.error(f"SFX file does not exist: {file_path}")
            return

        queued_sound = QueuedSoundEffect(
            event_key=event_key,
            file_path=file_path,
            volume=float(volume),
        )
        if interrupt:
            self._channel.stop()
            self._play_request(queued_sound)
            return

        self._queue.append(queued_sound)
        self.process_queue()

    def process_queue(self) -> None:
        """Start the next queued SFX once the dedicated channel is idle."""
        if self._channel.get_busy():
            return
        if not self._queue:
            return

        self._play_request(self._queue.popleft())

    def clear_queue(self) -> None:
        """Discard pending queued SFX requests."""
        self._queue.clear()

    def _play_request(self, queued_sound: QueuedSoundEffect) -> None:
        """Play one resolved SFX request immediately on the SFX channel."""
        try:
            sound = self._load_sound(queued_sound.file_path)
            sound.set_volume(queued_sound.volume)
            self._channel.play(sound)
            self._logger.info(f"audio SFX played: {queued_sound.event_key}")
        except Exception as error:  # pragma: no cover - runtime audio backend
            self._logger.error(
                f"failed to play SFX '{queued_sound.event_key}': {error}"
            )

    def _load_sound(self, file_path: str):
        """Load and cache a pygame sound object."""
        sound = self._cache.get(file_path)
        if sound is None:
            sound = self._mixer.Sound(file_path)
            self._cache[file_path] = sound
        return sound
