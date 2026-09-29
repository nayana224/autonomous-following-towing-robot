# ROS Interfaces

This document lists the operator-facing interfaces that other Autonomous Following and Towing Robot packages may rely on.

## Public topics

| Topic | Purpose |
| --- | --- |
| `/cmd_vel` | Robot velocity command |
| `/odom` | Robot odometry |
| `/scan` | Filtered LiDAR scan |
| `/map` | SLAM or localization map |
| `/amcl_pose` | Localization pose estimate |
| `/planned_path` | Saved or planned path visualization |
| `/follow_state` | Compatible follower state: `SEARCH`, `FOLLOW`, or `OBSTACLE` |
| `/tracking_detail` | Optional experimental-tracker diagnostics including lock state and target ID |
| `/mode_manager/status` | GUI workflow and safety status |
| `/mode_manager/audio_event` | Operator audio event |
| `/mode_manager/led_event` | Operator LED event |
| `/fall_detection/status` | Fall-detector observation state |
| `/fall_detection/detected` | Latched fall observation |
| `/fall_detection/heartbeat` | Detector liveness signal |

## Public mode-manager commands

- `/mode_manager/command/start_follow`
- `/mode_manager/command/start_recording_follow`
- `/mode_manager/command/finish_recording_for_alignment`
- `/mode_manager/command/start_alignment`
- `/mode_manager/command/skip_alignment`
- `/mode_manager/command/finish_alignment`
- `/mode_manager/command/prepare_autonomous`
- `/mode_manager/command/path_forward_auto`
- `/mode_manager/command/path_reverse_auto`
- `/mode_manager/command/stop`
- `/mode_manager/command/clear_error`
- `/mode_manager/command/clear_safety_stop`

The GUI must only call commands listed in the current `allowed_commands` field, except for explicitly internal startup or recovery actions.

## Fall-detection service

- `/fall_detection/reset`: clear the detector latch after the mode manager validates an operator safety-release request.

## `/mode_manager/status`

The status is a JSON text payload. GUI-facing fields include:

- `mode`
- `busy`
- `last_command`
- `last_error`
- `operator_step`
- `operator_message`
- `allowed_commands`
- `manual_control_allowed`
- `tracking_message`
- `recording_message`
- `path_message`
- `driving_message`
- `safety_stop_active`
- `safety_stop_in_progress`
- `safety_stop_result`
- `safety_interrupted_mode`
- `fall_detector_alive`
- `fall_detection_status`

## Internal interfaces

Services under `/mode_manager/internal/*` belong to runtime process management. The GUI must not use them for normal workflow control.

Path-manager services such as `/path_manager/start_record`, `/path_manager/stop_record`, `/path_manager/save_pose`, and `/path_manager/publish_initial_pose` are coordinated by `aftr_mode_manager`.

`/tracking_detail` is published only by the optional experimental tracker. Its `state` field can be `SEARCH`,
`LOCKED`, `AMBIGUOUS`, `OCCLUDED`, `LOST`, or `TF_UNAVAILABLE`. Consumers that
drive operator workflows must continue to use `/follow_state`.

## TF frames

The main frames are `map`, `odom`, `base_footprint`, `base_link`, and `laser`.
