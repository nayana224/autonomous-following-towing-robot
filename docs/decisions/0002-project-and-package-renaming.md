# ADR 0002: Project and Package Renaming

## Status

Accepted

## Context

The original MDBOT source is being adopted as a new project without importing its Git history. ADR 0001 records the former repository and package arrangement.

## Decision

- The project is named Autonomous Following and Towing Robot.
- The Git repository is `autonomous-following-towing-robot` under a ROS 2 workspace `src/` directory.
- All 13 first-party ROS 2 package identities change from `mdbot_*` to `aftr_*`.
- Package identity changes include manifests, build metadata, Python modules, package lookups, imports, and launch package arguments.
- Existing ROS runtime interfaces, node names, robot model identifiers, hardware plugin IDs, executable names, map names, and saved data formats remain unchanged.
- `laser_filters`, `serial-ros2`, and `sllidar_ros2` remain independent sibling repositories. External repository alignment and upstream/interface conventions are future work.

## Consequences

Build and launch commands use the new package names. Existing runtime integrations continue to use their established names and formats. The new repository retains its own Git history and does not import the old MDBOT history.
