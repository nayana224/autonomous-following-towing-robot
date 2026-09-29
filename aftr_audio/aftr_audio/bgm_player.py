# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors

"""Background-music playback helpers for AFTR audio feedback."""

from __future__ import annotations

import os


class BackgroundMusicPlayer:
    """Play and stop one looping background track at a time."""

    def __init__(self, mixer, logger) -> None:
        """Store the pygame mixer handle and ROS logger."""
        self._mixer = mixer
        self._logger = logger
        self._current_track_key: str | None = None

    @property
    def current_track_key(self) -> str | None:
        """Return the key of the currently active BGM track."""
        return self._current_track_key

    def start_loop(self, track_key: str, file_path: str, volume: float) -> None:
        """Start looping BGM unless the same track is already playing."""
        if self._current_track_key == track_key and self._mixer.music.get_busy():
            return
        if not os.path.exists(file_path):
            self._logger.error(f"BGM file does not exist: {file_path}")
            return

        try:
            self._mixer.music.load(file_path)
            self._mixer.music.set_volume(float(volume))
            self._mixer.music.play(loops=-1)
            self._current_track_key = track_key
            self._logger.info(f"audio BGM started: {track_key}")
        except Exception as error:  # pragma: no cover - runtime audio backend
            self._current_track_key = None
            self._logger.error(f"failed to start BGM '{track_key}': {error}")

    def stop(self) -> None:
        """Stop the active BGM track when one is running."""
        if self._current_track_key is None and not self._mixer.music.get_busy():
            return

        stopped_key = self._current_track_key or "unknown"
        try:
            self._mixer.music.stop()
            self._current_track_key = None
            self._logger.info(f"audio BGM stopped: {stopped_key}")
        except Exception as error:  # pragma: no cover - runtime audio backend
            self._logger.error(f"failed to stop BGM '{stopped_key}': {error}")
