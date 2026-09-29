# Validation Checklist

Run focused tests after changing workflow, GUI, safety, SLAM, Nav2, or path recording.

## Laptop CPU Docker baseline

Phase 1 was verified on an Ubuntu 26 LTS amd64 host using the Ubuntu 22.04 / ROS 2 Humble CPU-only image. Linux amd64 with Docker is the intended development setup; other host distributions have not been verified.

From the Container's `/workspace` after the Laptop Build:

```bash
colcon --log-base log_laptop test --build-base build_laptop --install-base install_laptop --parallel-workers 2
colcon test-result --test-result-base build_laptop --verbose
```

| Check | Phase 1 result |
| --- | --- |
| Docker image and ROS 2 Humble | Passed |
| Architecture | `x86_64` |
| Package discovery | 13 `aftr_*`, 3 external packages, no `mdbot_*` package identity |
| Whole Workspace Build | 16 packages passed |
| CPU-only PyTorch | `2.2.2+cpu`; `torch.version.cuda is None`; `torch.cuda.is_available() is False` |
| Python import | `aftr_fall_detection` and six other node modules passed |
| Launch / YAML syntax | 10 / 7 passed |
| Tests | **89 passed, 4 skipped, 4 failed** |

The four failures are existing lint/docstring checks: `aftr_path_manager` (2), `aftr_status_led` (1), and `aftr_mode_manager` (1). No test failure was attributed to missing hardware or CPU dependencies. The fall detector import and CPU fallback path passed, but actual model inference was not run because the model file was unavailable.

Laptop CI excludes physical motor, LiDAR, RealSense, GPIO, GPU inference, and the default operator-system hardware launch. Those checks belong to Jetson integration. This baseline records the Phase 1 run; rerun the commands above for current results.

The following sections retain the existing manual and robot-hardware validation checklist. They are separate from the Laptop CPU Docker commands above.

## Build

```bash
source /opt/ros/humble/setup.bash
export AFTR_WS=~/autonomous_following_towing_robot_ws
cd "$AFTR_WS"
colcon build --symlink-install --packages-select <changed_packages>
```

For `aftr_fall_detection`, activate the GPU virtual environment and run
`python3 -m colcon build --symlink-install --packages-select
aftr_fall_detection`. Do not use bare `/usr/bin/colcon`, because it generates
a system-Python console script that cannot import the virtual-environment GPU
packages.

## GUI

- Main window opens without missing `.ui` files.
- All visible buttons have readable foreground and background colors.
- Buttons are disabled while a command is running.
- The joystick accepts touches across the outer circle.
- Touch release returns the joystick input to neutral.
- Page changes do not leave stale joystick input active.

## Follow and recording

- Standard follow starts and stops repeatedly.
- Recording follow starts SLAM before path recording and follower motion.
- A fresh `/map` is received for the current recording session.
- Recording stop saves the path and pose.
- Map-save failure is reported without leaving unsafe moving processes active.
- A second recording session can start after returning HOME.

## Localization and replay

- SLAM stops before Nav2 localization starts.
- Nav2 readiness and lifecycle states are confirmed before initial-pose publication.
- Autonomous replay starts only from an allowed state.
- Stop interrupts replay and returns to a stable state.

## Fall safety

- A confirmed fall activates the safety latch.
- Partial pose does not accumulate toward a confirmed fall.
- Detector `False` alone does not clear the safety latch.
- Detector heartbeat loss during an active workflow starts safety cleanup.
- Safety release is rejected while the detector is unhealthy or still reports `FALL_DETECTED`.
- Successful release returns HOME and does not resume the previous task.

## Repeated scenario

Run this sequence at least three times:

1. Start recording follow.
2. Trigger safety stop.
3. Clear safety stop.
4. Start recording follow again.
5. Finish recording normally.
6. Complete alignment and autonomous preparation.

Record the full ROS log whenever a step enters `ERROR` or a managed process requires forced termination.
