# Autonomous Following and Towing Robot Path Manager Code Reading Guide

This document is a short guide for reading the `aftr_path_manager` package.

## Recommended Reading Order

1. `aftr_path_manager/path_manager_node.py`
2. `aftr_path_manager/recording/path_recording.py`
3. `aftr_path_manager/follow_runtime/state.py`
4. `aftr_path_manager/path_safety_cleanup.py`
5. `aftr_path_manager/replay_cleanup/*.py`
6. `aftr_path_manager/path_storage.py`
7. `aftr_path_manager/path_geometry.py`
8. `aftr_path_manager/path_types.py`

## Package Structure

### `path_manager_node.py`
- Main ROS 2 node
- Owns services, subscriptions, publishers, TF lookup, and Nav2 action calls
- Entry point for recording, saving pose, loading path, replay, cancel, and status publishing

### `recording/path_recording.py`
- Recording-specific helper functions
- Decides whether a new pose should be recorded
- Filters global-frame poses
- Simplifies the recorded path before writing CSV

### `follow_runtime/state.py`
- FollowPath runtime state and helper functions
- Stores retry state, active replay path state, and remaining distance helpers
- Keeps saved-path replay bookkeeping out of the ROS node body

### `path_safety_cleanup.py`
- Facade for replay-path cleanup
- Runs the replay cleanup pipeline before Nav2 replay
- Connects the node-level parameter set to the lower-level cleanup modules

### `replay_cleanup/`
- Low-level replay path cleanup logic
- `grid_utils.py`: occupancy-grid and clearance helpers
- `path_context.py`: local corridor and corner context classification
- `candidate_search.py`: lateral candidate generation and scoring
- `shift_stabilizer.py`: shift smoothing and corridor stabilization
- `corner_smoothing.py`: corner detection and curve smoothing

### `path_storage.py`
- CSV and YAML load/save helpers
- Keeps file format handling separate from node logic

### `path_geometry.py`
- Shared geometric helper functions
- Angle normalization, nearest index search, quaternion/yaw conversions, and distance helpers

### `path_types.py`
- Shared lightweight data types
- Currently used for compact recorded path pose representation

## How to Read the Main Node

Inside `path_manager_node.py`, read the file in this order:

1. Initialization
2. Status publishing
3. Service callbacks
4. Recording workflow
5. Nav2 path replay
6. FollowPath retry handling
7. Replay preprocessing
8. Shutdown

## What Is Public vs Internal

### Public behavior
- ROS services exposed by `PathManagerNode`
- Published status format on `~/status`
- Saved path CSV and saved pose YAML behavior

### Internal implementation
- Replay cleanup algorithm details
- Retry delay calculation
- Recording simplification rules
- Internal helper split across `recording/`, `follow_runtime/`, and `replay_cleanup/`

## Refactoring Notes

- Keep `path_manager_node.py` focused on ROS communication and orchestration
- Keep recording logic out of the main ROS node where possible
- Keep replay cleanup logic out of service callbacks
- Keep behavior-changing tuning separate from structure-only refactoring
