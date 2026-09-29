# Architecture

Autonomous Following and Towing Robot is a ROS 2 Humble mobile-robot system for worker following, path recording, alignment, localization, and recorded-path replay.

## Ownership

- `aftr_gui` renders operator state and sends allowed commands.
- `aftr_mode_manager` owns workflow state, command serialization, process startup order, readiness checks, and safety cleanup.
- `aftr_fall_detection` publishes fall observations and detector heartbeat.
- `aftr_tracking` follows the selected worker.
- `aftr_path_manager` records, saves, and replays paths and poses.
- `aftr_slam` provides mapping.
- `aftr_navigation` provides localization and Nav2 replay.
- `aftr_audio` and `aftr_status_led` render operator feedback events.

## Normal workflow

```text
HOME
→ FOLLOW or RECORDING_FOLLOW
→ ALIGNMENT_DECISION
→ ALIGNMENT or LOCALIZING
→ AUTONOMOUS_READY
→ AUTONOMOUS_DRIVING
```

Recording-follow startup is ordered conservatively:

```text
base ready
→ SLAM start
→ fresh /map received
→ path manager ready
→ recording active
→ follower start
```

Localization preparation is also sequential:

```text
recording stop
→ path/map/pose save
→ SLAM stop confirmed
→ Nav2 start
→ lifecycle readiness confirmed
→ initial pose publish
→ AMCL pose confirmed
```

## GUI design

Static layouts belong in Qt Designer files under `aftr_gui/gui/`.

- `mdbot_gui.ui`: main operator window and normal workflow pages
- `safety_stop.ui`: safety-stop page
- `safety_clear_confirm.ui`: safety-release confirmation dialog

Python code loads these files, binds signals, and updates dynamic state. It should not rebuild equivalent temporary layouts in code.

## Command rules

- The GUI uses `allowed_commands` from `/mode_manager/status`.
- The mode manager rejects overlapping commands through one command lock.
- Heavy processes are started and stopped in sequence with readiness checks.
- A failed startup rolls back processes started by that command.
- `/cmd_vel` ownership must remain unambiguous between manual alignment, tracking, and autonomous driving.

## Safety workflow

```text
fall or detector loss
→ safety latch
→ active workflow cleanup
→ SAFETY_STOP page
→ operator checks worker and environment
→ clear_safety_stop
→ detector reset
→ HOME / IDLE
```

The previous workflow is never resumed automatically after safety release.
