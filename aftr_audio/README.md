# aftr_audio

`aftr_audio` is a standalone ROS 2 package for Autonomous Following and Towing Robot audio feedback.

## Responsibility

- Subscribe to `/mode_manager/audio_event`
- Play looping BGM and one-shot SFX without blocking ROS callbacks
- Apply cooldown rules, queued SFX playback, and duplicate-event protection

This package does not decide robot workflow state transitions.

## Topic

- `/mode_manager/audio_event` (`std_msgs/msg/String`)

## Supported Events

- `autonomous_start`
- `system_ready`
- `autonomous_ready`
- `autonomous_bgm_start`
- `autonomous_bgm_stop`
- `arrived`
- `blocked`
- `failed`
- `follow_start`
- `recording_follow_start`
- `worker_detected`
- `worker_lost`
- `alignment_request`
- `alignment_confirmation_request`
- `alignment_completed`
- `follow_end`
- `recording_follow_completed`
- `return_to_home`

## Package Layout

```text
aftr_audio/
  aftr_audio/
    audio_node.py
    audio_policy.py
    bgm_player.py
    sfx_player.py
  config/
    audio_events.yaml
  sounds/
    *.wav
  launch/
    audio.launch.py
```

## Parameters

- `enabled`
- `config_file`
- `sounds_dir`
- `bgm_volume_scale`
- `sfx_volume_scale`

## Launch

The commands below use the manual `build/install/log` layout. For Laptop Docker, use [the development guide](../docs/development/laptop-docker.md).

```bash
cd ~/autonomous_following_towing_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch aftr_audio audio.launch.py
```

## Manual Test Commands

```bash
ros2 topic pub --once /mode_manager/audio_event std_msgs/msg/String "{data: 'autonomous_start'}"
```

```bash
ros2 topic pub --once /mode_manager/audio_event std_msgs/msg/String "{data: 'autonomous_bgm_start'}"
```

```bash
ros2 topic pub --once /mode_manager/audio_event std_msgs/msg/String "{data: 'blocked'}"
```

```bash
ros2 topic pub --once /mode_manager/audio_event std_msgs/msg/String "{data: 'arrived'}"
```

```bash
ros2 topic pub --once /mode_manager/audio_event std_msgs/msg/String "{data: 'failed'}"
```

## Notes

- `autonomous_moving_bgm.wav` is treated as looping BGM.
- `arrived` and `failed` stop the active BGM before playing SFX.
- `blocked` uses cooldown protection to avoid repeated playback spam.
- SFX defaults to queued playback so back-to-back prompts do not cut each other off.
- Current repository configuration keeps workflow SFX on queued playback to
  avoid prompt truncation on slower systems.
- Follow-mode cues cover both `FOLLOW` and `RECORDING_FOLLOW`.
- `worker_lost` is mapped to `follow_worker_searching.wav`.
- `recording_follow_completed` replaces the older split sequence of follow-end plus save-complete prompts.
- `alignment_confirmation_request` is delayed briefly after arrival so the arrival cue finishes cleanly first.
- `return_to_home` is the shared cue for operator-driven cancel and home-return flows.
- `autonomous_ready` must play whether the workflow reaches
  `AUTONOMOUS_READY` immediately or after a `LOCALIZING` wait.
- Audio backend failures should not block robot behavior.
