# Safety

This document describes the implemented safety policy and the items that still require hardware validation.

## Implemented policy

- A confirmed fall activates a latched safety stop.
- Detector `False` does not clear the latch automatically.
- The operator must clear the latch through `/mode_manager/command/clear_safety_stop`.
- Safety release is rejected while cleanup is running, the detector heartbeat is unavailable, or the detector still reports `FALL_DETECTED`.
- Successful release resets the detector and returns the workflow to HOME/IDLE.
- The interrupted task is not resumed automatically.
- Detector heartbeat loss during an active workflow triggers safety cleanup.
- Camera-frame timeout also suppresses the detector heartbeat and therefore follows the same latched cleanup path.
- New movement commands are blocked while the detector is unavailable.

## Cleanup priority

Safety cleanup prioritizes stopping motion-producing processes.

Critical failures include a follower, Nav2, path-manager, or SLAM process that cannot be stopped when required for a safe state. A map-save failure also blocks autonomous preparation so a newly recorded path cannot be paired with an older map.

## GUI behavior

The safety page shows:

- captured camera image
- cleanup progress
- interrupted mode
- detector connection state
- current detector observation

The release action requires a confirmation dialog. Button availability in the GUI is only advisory; the mode manager performs the final safety validation.

## Fall-detection quality

A pose must contain reliable head, shoulder, and hip observations before normal fall classification. Upper-body clipping, low-confidence critical keypoints, and partial poses prevent fall-candidate accumulation.

## General rules

- Stop actions take priority over workflow progression.
- Manual control and autonomous replay must not own `/cmd_vel` at the same time.
- GUI joystick authority is enabled only on the initial HOME screen and during active manual alignment, where no autonomous command producer owns `/cmd_vel`.
- Autonomous replay must not start before localization readiness is confirmed.
- Changes affecting braking, motor communication, obstacle sensing, or command ownership require real-robot validation.

## Hardware validation still required

- final stopping distance after joystick release
- stopping behavior after motor-driver communication loss
- exact detector-heartbeat timeout under camera or GPU overload
- towing stability under acceleration and turning
- behavior after complete GUI disconnection
