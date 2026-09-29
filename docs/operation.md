# Operator Guide

This document describes the normal touchscreen workflow.

## Start

Launch the operator system and wait until the base system is ready.

## Follow

1. Leave `경로 저장` unchecked.
2. Press `작업자 추종`.
3. Press the stop or back button to end following.

## Recording follow

1. Enable `경로 저장`.
2. Press `작업자 추종`.
3. The system starts SLAM, waits for a fresh map, starts the path manager, confirms recording, and then starts the follower.
4. Press the recording-finish button to save the path and continue to alignment preparation.
5. Autonomous preparation stops if a new, complete map pair cannot be verified. An older map is never accepted as the result of the current recording.

## Alignment

Use the large virtual joystick to align the robot.

- The whole outer joystick circle is touchable.
- The knob moves directly to the touched position.
- Releasing the joystick returns the input to neutral. Actual deceleration is handled by the robot control stack.
- Press `정렬 완료` after positioning is complete.

## Autonomous replay

When localization and Nav2 are ready, press `이전 위치로 이동` to replay the saved path. Use the stop button to interrupt driving.

## Safety stop

When a fall or detector-loss safety event occurs:

1. The active workflow is stopped.
2. The safety page displays the captured camera image and safety status.
3. Check the worker and surrounding area directly.
4. Press `안전 정지 해제`.
5. Confirm the dialog.
6. The system returns to HOME without automatically resuming the previous task.

## Button behavior

- Stop actions do not require confirmation.
- Safety release requires confirmation.
- Buttons are disabled while another command is being processed.
- The GUI only enables commands listed by `mode_manager` in `allowed_commands`.
- The virtual joystick publishes commands on the initial HOME screen and during active manual alignment.
- Follow, recording, localization, autonomous driving, error, and safety states disable manual joystick commands.

## Runtime recovery

- The camera, fall detector, audio node, and status LED node are respawned by the operator launch if their process exits unexpectedly.
- A temporary fall-detector restart blocks new movement until its heartbeat returns.
- The detector heartbeat is withheld when camera frames become stale, so a camera-only failure also stops an active workflow and cannot resume it automatically.
- Managed motion processes converge to `ERROR` after an unexpected exit and never resume movement automatically.
- Error clear returns to HOME only after follower, Nav2, path-manager, and SLAM process groups are confirmed stopped.

## Mapping without the fall camera

For a controlled SLAM diagnostic run, disable both the RealSense process and
fall detection explicitly:

```bash
ros2 launch aftr_gui operator_system.launch.py \
  enable_fall_camera:=false \
  enable_fall_detection:=false
```

This mode keeps the GUI, tracking, LiDAR, SLAM, and path recording available,
but fall-detection safety is unavailable for the entire run.
