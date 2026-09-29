# ADR 0001: Repository and Package Structure Baseline

## Status

Accepted

## Context

MDBOT is developed as a ROS 2 Humble project inside a larger workspace. The `mdbot` repository contains multiple first-party ROS 2 packages, while some required sibling packages are external and not owned by this repository.

The project also needs a stable documentation baseline before code refactoring can be done safely.

## Decision

- `mdbot` is managed as the Git repository root.
- The repository lives inside a ROS 2 workspace rather than acting as the workspace root.
- External sibling packages such as `laser_filters`, `serial-ros2`, and `sllidar_ros2` are not part of this repository.
- The project uses multiple ROS 2 packages with explicit package boundaries.
- Documentation is treated as the baseline before structural code refactoring.

## Consequences

- Repository-level documentation should describe the package structure clearly.
- Public ROS interfaces should be documented before large refactoring work.
- Cross-package cleanup should be incremental rather than broad and simultaneous.
- Hardware-sensitive packages should remain conservative and isolated from workflow refactoring.
