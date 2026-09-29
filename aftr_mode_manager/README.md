# aftr_mode_manager

`aftr_mode_manager` owns the operator workflow, command serialization, managed-process order, readiness checks, recovery, and fall-safety latch.

## Main responsibilities

- start and stop base, SLAM, path manager, follower, Nav2, and related runtime processes
- expose operator-facing command services
- publish `/mode_manager/status`
- reject overlapping commands
- roll back partially started workflows
- coordinate safety cleanup and operator-approved release

## Main workflow

```text
IDLE
→ FOLLOW or RECORDING_FOLLOW
→ ALIGNMENT
→ LOCALIZING
→ AUTONOMOUS_READY
→ AUTONOMOUS_DRIVING
```

Recording-follow startup is ordered as follows:

```text
base ready
→ SLAM ready
→ fresh /map
→ path manager ready
→ recording active
→ follower start
```

Localization startup stops SLAM first, confirms process shutdown, starts Nav2, waits for lifecycle readiness, publishes the initial pose, and confirms AMCL output.

Recording completion requires the map YAML and referenced image to be a new, complete pair. Map-save failure stops the workflow before localization. Error reset also verifies that every upper-layer managed process group exited before returning to `IDLE`.

## Public commands

The main services are under `/mode_manager/command/*`, including follow, recording follow, alignment, autonomous preparation, replay, stop, error clear, and safety-stop clear.

Low-level process services under `/mode_manager/internal/*` are implementation details and should not be called by the GUI during normal operation.

## Safety

A fall or detector-heartbeat loss can activate a latched safety stop. The interrupted task is stopped and is not resumed automatically. Release requires `/mode_manager/command/clear_safety_stop` and final validation by the mode manager.

## Build

```bash
source /opt/ros/humble/setup.bash
cd ~/autonomous_following_towing_robot_ws
colcon build --symlink-install --packages-select aftr_mode_manager
```

## Related documentation

- [Architecture](../docs/architecture.md)
- [ROS interfaces](../docs/interfaces.md)
- [Safety](../docs/safety.md)
- [Validation checklist](../docs/testing.md)
