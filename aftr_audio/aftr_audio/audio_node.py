# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors

"""ROS 2 node that plays AFTR audio feedback events."""

from __future__ import annotations

from ament_index_python.packages import get_package_share_directory
import os
import traceback

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml

from aftr_audio.audio_policy import AudioPolicy
from aftr_audio.audio_policy import SFX_PLAYBACK_MODE_INTERRUPT
from aftr_audio.bgm_player import BackgroundMusicPlayer
from aftr_audio.sfx_player import SoundEffectPlayer


class AudioNode(Node):
    """Subscribe to mode-manager audio events and play WAV feedback."""

    def __init__(self) -> None:
        """Initialize parameters, config, audio backend, and ROS interfaces."""
        super().__init__("mdbot_audio")
        self._declare_parameters()
        self._load_configuration()
        self._validate_sound_assets()
        self._initialize_audio_backend()
        self.create_subscription(
            String,
            "/mode_manager/audio_event",
            self.audio_event_callback,
            20,
        )
        self.create_timer(0.05, self._process_sfx_queue)
        self.get_logger().info(
            "mdbot_audio ready "
            f"(enabled={self.enabled}, audio_backend_ready={self.audio_backend_ready})"
        )

    def _declare_parameters(self) -> None:
        """Declare ROS parameters for configuration and runtime control."""
        share_dir = get_package_share_directory("aftr_audio")
        default_config = os.path.join(share_dir, "config", "audio_events.yaml")
        default_sounds_dir = os.path.join(share_dir, "sounds")

        self.enabled = bool(self.declare_parameter("enabled", True).value)
        self.config_file = str(
            self.declare_parameter("config_file", default_config).value
        )
        self.sounds_dir = str(
            self.declare_parameter("sounds_dir", default_sounds_dir).value
        )
        self.bgm_volume_scale = float(
            self.declare_parameter("bgm_volume_scale", 1.0).value
        )
        self.sfx_volume_scale = float(
            self.declare_parameter("sfx_volume_scale", 1.0).value
        )

    def _load_configuration(self) -> None:
        """Load the YAML event mapping and build the audio policy."""
        config = self._read_audio_config(self.config_file)
        self.audio_policy = AudioPolicy.from_config(
            config=config,
            bgm_volume_scale=self.bgm_volume_scale,
            sfx_volume_scale=self.sfx_volume_scale,
        )

    def _initialize_audio_backend(self) -> None:
        """Initialize pygame mixer and players without blocking robot behavior."""
        self.audio_backend_ready = False
        self.bgm_player = None
        self.sfx_player = None
        self._pygame = None

        if not self.enabled:
            self.get_logger().warn("mdbot_audio is disabled by parameter")
            return

        try:
            import pygame

            self._initialize_pygame_mixer(pygame)
            pygame.mixer.set_num_channels(8)
            self._pygame = pygame
            self.bgm_player = BackgroundMusicPlayer(pygame.mixer, self.get_logger())
            self.sfx_player = SoundEffectPlayer(
                pygame.mixer,
                self.get_logger(),
                channel_index=0,
            )
            self.audio_backend_ready = True
        except Exception as error:  # pragma: no cover - runtime audio backend
            self.audio_backend_ready = False
            self.get_logger().error(f"failed to initialize audio backend: {error}")

    def _validate_sound_assets(self) -> None:
        """Log missing sound assets early so launch-time issues are visible."""
        missing_assets: list[str] = []

        for bgm_key, bgm_config in self.audio_policy.iter_bgm_configs():
            _, resolved_path = self._resolve_sound_path(
                bgm_config.file_name,
                bgm_config.fallback_files,
            )
            if resolved_path is None:
                missing_assets.append(f"bgm:{bgm_key}->{bgm_config.file_name}")

        for sfx_key, sfx_config in self.audio_policy.iter_sfx_configs():
            _, resolved_path = self._resolve_sound_path(
                sfx_config.file_name,
                sfx_config.fallback_files,
            )
            if resolved_path is None:
                missing_assets.append(f"sfx:{sfx_key}->{sfx_config.file_name}")

        if missing_assets:
            self.get_logger().error(
                "missing audio assets in sounds_dir "
                f"'{self.sounds_dir}': {', '.join(missing_assets)}"
            )
            return

        self.get_logger().info(
            f"audio assets validated from sounds_dir: {self.sounds_dir}"
        )

    def _initialize_pygame_mixer(self, pygame) -> None:
        """Initialize pygame mixer, falling back to a dummy backend if needed."""
        try:
            pygame.mixer.init()
            return
        except Exception as first_error:
            self.get_logger().warn(
                "default audio backend failed; retrying with SDL dummy driver: "
                f"{first_error}"
            )

        previous_driver = os.environ.get("SDL_AUDIODRIVER")
        os.environ["SDL_AUDIODRIVER"] = "dummy"
        try:
            pygame.mixer.init()
            self.get_logger().warn(
                "mdbot_audio is running with SDL dummy audio driver"
            )
        except Exception:
            if previous_driver is None:
                os.environ.pop("SDL_AUDIODRIVER", None)
            else:
                os.environ["SDL_AUDIODRIVER"] = previous_driver
            raise

    def _read_audio_config(self, config_file: str) -> dict:
        """Read the audio YAML config or return an empty fallback config."""
        if not os.path.exists(config_file):
            self.get_logger().error(f"audio config file does not exist: {config_file}")
            return {}

        try:
            with open(config_file, "r", encoding="utf-8") as config_stream:
                data = yaml.safe_load(config_stream) or {}
            if not isinstance(data, dict):
                self.get_logger().error("audio config must be a YAML mapping")
                return {}
            return data
        except Exception as error:
            self.get_logger().error(f"failed to load audio config: {error}")
            return {}

    def audio_event_callback(self, msg: String) -> None:
        """Handle one incoming audio event without blocking robot execution."""
        event_name = msg.data.strip()
        if not event_name:
            return

        if not self.enabled:
            return

        if not self.audio_backend_ready:
            self.get_logger().warn(
                f"audio backend is unavailable; ignored event: {event_name}"
            )
            return

        actions = self.audio_policy.build_actions(
            event_name=event_name,
            current_bgm_key=self.bgm_player.current_track_key,
            logger=self.get_logger(),
        )
        for action in actions:
            self._execute_audio_action(
                action.kind,
                action.key,
                action.playback_mode,
            )

    def _execute_audio_action(
        self,
        action_kind: str,
        action_key: str,
        playback_mode: str = "",
    ) -> None:
        """Execute one audio action resolved by the policy layer."""
        if action_kind == "start_bgm":
            self._start_bgm(action_key)
            return
        if action_kind == "stop_bgm":
            self.bgm_player.stop()
            return
        if action_kind == "play_sfx":
            self._play_sfx(action_key, playback_mode)
            return

        self.get_logger().warn(f"unsupported audio action ignored: {action_kind}")

    def _start_bgm(self, bgm_key: str) -> None:
        """Start one configured background-music track."""
        bgm_config = self.audio_policy.get_bgm_config(bgm_key)
        if bgm_config is None:
            self.get_logger().warn(f"missing BGM config for key: {bgm_key}")
            return

        file_name, file_path = self._resolve_sound_path(
            bgm_config.file_name,
            bgm_config.fallback_files,
        )
        if file_path is None:
            self.get_logger().error(
                f"BGM file does not exist for '{bgm_key}': {bgm_config.file_name}"
            )
            return
        self._log_fallback_file_use(bgm_config.file_name, file_name, bgm_key, "BGM")
        self.bgm_player.start_loop(
            track_key=bgm_key,
            file_path=file_path,
            volume=bgm_config.volume,
        )

    def _play_sfx(self, sfx_key: str, playback_mode: str) -> None:
        """Play one configured sound effect."""
        sfx_config = self.audio_policy.get_sfx_config(sfx_key)
        if sfx_config is None:
            self.get_logger().warn(f"missing SFX config for key: {sfx_key}")
            return

        file_name, file_path = self._resolve_sound_path(
            sfx_config.file_name,
            sfx_config.fallback_files,
        )
        if file_path is None:
            self.get_logger().error(
                f"SFX file does not exist for '{sfx_key}': {sfx_config.file_name}"
            )
            return
        self._log_fallback_file_use(sfx_config.file_name, file_name, sfx_key, "SFX")
        self.sfx_player.enqueue(
            event_key=sfx_key,
            file_path=file_path,
            volume=sfx_config.volume,
            interrupt=(playback_mode == SFX_PLAYBACK_MODE_INTERRUPT),
        )

    def _process_sfx_queue(self) -> None:
        """Advance the queued SFX channel without blocking ROS callbacks."""
        if not self.enabled or not self.audio_backend_ready or self.sfx_player is None:
            return
        self.sfx_player.process_queue()

    def _resolve_sound_path(
        self,
        primary_file_name: str,
        fallback_files: tuple[str, ...],
    ) -> tuple[str, str | None]:
        """Return the first existing sound file path from primary and fallbacks."""
        candidate_files = (primary_file_name, *fallback_files)
        for file_name in candidate_files:
            file_path = os.path.join(self.sounds_dir, file_name)
            if os.path.exists(file_path):
                return file_name, file_path
        return primary_file_name, None

    def _log_fallback_file_use(
        self,
        expected_file_name: str,
        resolved_file_name: str,
        event_key: str,
        asset_type: str,
    ) -> None:
        """Warn when playback had to fall back to a compatibility file name."""
        if resolved_file_name == expected_file_name:
            return
        self.get_logger().warn(
            f"{asset_type} '{event_key}' used fallback file "
            f"'{resolved_file_name}' instead of '{expected_file_name}'"
        )

    def destroy_node(self) -> bool:
        """Shut down the audio backend before destroying the ROS node."""
        try:
            if self.bgm_player is not None:
                self.bgm_player.stop()
            if self.sfx_player is not None:
                self.sfx_player.clear_queue()
            if self._pygame is not None:
                self._pygame.mixer.quit()
        except Exception as error:  # pragma: no cover - runtime audio backend
            self.get_logger().warn(f"audio backend shutdown warning: {error}")
        return super().destroy_node()


def main(args=None) -> None:
    """Run the AFTR audio node."""
    rclpy.init(args=args)
    node = None

    try:
        node = AudioNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:  # pragma: no cover - top-level runtime guard
        traceback.print_exc()
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
