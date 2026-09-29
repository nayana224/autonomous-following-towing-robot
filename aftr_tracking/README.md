# Autonomous Following and Towing Robot Tracking

`aftr_tracking` detects worker candidates from `/scan` and follows one locked
worker. The default tracker uses the field-tested nearest-target behavior from
`aftr_tracking/tracker_node.py`. The normal integration entry point is the complete
operator system, not this package's launch file by itself.

## Tracking behavior

- Initial worker search uses a ±50 degree field of view and requires a candidate
  for 20 scan frames.
- Follow mode uses the original ±80 degree field of view.
- The nearest candidate is locked, then updated only when its displacement from
  the previous target is less than `0.55 m`.
- A missing or unmatched candidate retains the last target for up to 16 frames
  before returning to `SEARCH`.
- The following distance is `0.47 m`, with the original acceleration, braking,
  TTC, and between-robot-and-worker obstacle checks preserved.

The public `/follow_state` values remain `SEARCH`, `FOLLOW`, and `OBSTACLE` for
compatibility. The mode manager converts those states into voice and LED events;
the tracking node does not start its own audio process.

## Build

The commands below use the manual `build/install/log` layout. For Laptop Docker, use [the development guide](../docs/development/laptop-docker.md). Run builds from the ROS workspace root:

```bash
cd ~/autonomous_following_towing_robot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  aftr_tracking aftr_mode_manager aftr_gui aftr_status_led
source install/setup.bash
```

Re-source `install/setup.bash` in every new terminal. A package-local source
directory is not a valid runtime environment because the mode manager starts
the installed launch and console-script entries.

## Run through the operator system

Start the complete system with the stable tracker (the default):

```bash
cd ~/autonomous_following_towing_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch aftr_gui operator_system.launch.py
```

The operator launch starts the mode manager, GUI, audio, status LED, and fall
safety components. The tracking process itself starts when the operator selects
a follow workflow; it is not expected to be present while the robot is idle.

The experimental motion-prediction tracker remains available for comparison,
but is not the default:

```bash
ros2 launch aftr_gui operator_system.launch.py robust_tracking:=true
```

This switch changes only the tracking executable. Normal field tests should omit
it. Neither mode changes the motor driver, `ros2_control`, mapping, navigation,
or the MD400T firmware settings.

## Observe and tune

Useful runtime checks are:

```bash
ros2 topic echo /follow_state
ros2 topic echo /cmd_vel
```

The default tracker does not publish `/tracking_detail`; that topic belongs to
the optional experimental tracker. For the stable tracker, use RViz person
markers together with `/target_person`, `/follow_state`, and `/cmd_vel`.

The mode manager delays only the red LED transition for a brief obstacle pulse
(`follow_obstacle_led_hold_sec`, default `0.5`). It adds no delay to the
follower's own stop decision. Clearing the red indication is also debounced by
`follow_obstacle_clear_hold_sec` (default `0.4`).

## No-hardware tests

```bash
cd ~/autonomous_following_towing_robot_ws
source /opt/ros/humble/setup.bash
colcon test --packages-select aftr_tracking
colcon test-result --test-result-base build/aftr_tracking --verbose
python3 -m pytest \
  src/autonomous-following-towing-robot/aftr_mode_manager/test/test_follow_state_debounce.py -q
ros2 launch aftr_gui operator_system.launch.py --show-args
```

Before a moving test, lift the drive wheels or keep an emergency-stop operator
ready. Verify initial confirmation, normal turning, brief leg occlusion, obstacle
stop, worker loss after 16 frames, voice transitions, and LED transitions.
