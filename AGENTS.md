# AGENTS.md

## Project

- Project: Autonomous Following and Towing Robot (ROS 2 Humble).
- Repository: `autonomous-following-towing-robot`; ROS package namespace: `aftr_*`.
- The directory containing this `AGENTS.md` is the Git repository root.
- This repository lives under `src/` in a ROS 2 workspace. Its workspace root is two directories above the repository root.
- Run `colcon build`, `colcon test`, and `colcon test-result` from the workspace root.

## Workspace Rules

- Do not edit or commit generated workspace outputs such as `build/`, `install/`, `log/`, or Python caches.
- External sibling packages in the workspace, including `laser_filters`, `serial-ros2`, and `sllidar_ros2`, are not part of this repository and must not be modified unless the user explicitly asks for it.
- Keep shared project documentation under `docs/`; preserve package-specific notes where they provide distinct implementation context.
- Keep original manuals, PDFs, and other evidence under `docs/references/`.
- Use repository-relative paths for images and documentation links in `README.md`; verify targets exist.
- Keep documentation commands consistent with the actual Dockerfile and scripts; avoid repeating long setup instructions across `README.md` and `docs/`.

## Package Map

- `aftr_description`: robot description assets such as URDF, meshes, and visualization support files.
- `aftr_hardware`: `ros2_control` hardware interface and MD motor driver communication.
- `aftr_bringup`: base robot bringup, controller setup, LiDAR driver startup, and low-level runtime configuration.
- `aftr_teleop`: manual teleoperation tools for `/cmd_vel`.
- `aftr_tracking`: target tracking logic and tracking-related ROS interfaces.
- `aftr_path_manager`: path recording, path replay, pose persistence, and Nav2 path-following integration.
- `aftr_slam`: SLAM Toolbox launch and mapping configuration.
- `aftr_navigation`: Nav2 launch, map usage, and navigation configuration.
- `aftr_mode_manager`: workflow state machine, managed process orchestration, and operator-facing status.
- `aftr_gui`: operator GUI, ROS communication for the GUI, and passive rendering of mode-manager state.
- `aftr_fall_detection`: fall observations and detector heartbeat.
- `aftr_audio`: voice and sound feedback.
- `aftr_status_led`: LED feedback.

## Build and Test

For normal setup and Docker usage, follow `docs/setup.md`. Keep test, lint, troubleshooting, and development-only commands in `docs/maintenance.md`.

- Source ROS before ROS-related commands when needed: `source /opt/ros/humble/setup.bash`.
- Source the workspace after building when needed: `source install/setup.bash`.
- Build a focused package with `colcon build --symlink-install --packages-select <package>`.
- Run focused tests with `colcon test --packages-select <package>`.
- Check results with `colcon test-result --verbose`.

## Docker and Architecture Boundaries

- Laptop development and CI use the Ubuntu 22.04 / ROS 2 Humble CPU-only Docker image in `docker/laptop/`.
- The laptop run script enables X11/XWayland GUI forwarding by default when a usable Host display exists; use `--headless` for CI or explicit offscreen runs. GUI access does not imply CUDA or GPU compute support.
- Keep a CJK font dependency in the laptop image for Korean GUI text; prefer system font fallback over OS-specific font families in Qt source and UI files.
- Container applications must run with the Host UID/GID. The runtime must provide valid passwd/group entries and a writable HOME for that identity.
- Use narrow display access instead of a privileged container; GUI infrastructure changes must preserve ROS runtime interfaces.
- Keep CUDA, TensorRT, NVIDIA Container Runtime, Jetson.GPIO, and Jetson-specific packages out of the laptop image and shared dependency lists.
- Jetson Orin Nano Super uses `docker/jetson/` with NVIDIA's Jetson L4T CUDA 12.6 runtime image, JetPack 6 CUDA 12.6 arm64 PyTorch wheels, and ROS 2 Humble on Jammy. Do not use the NVIDIA PyTorch 24.10 iGPU container: it requires driver 560+, while this robot uses driver 540.5.0. The Jetson image has passed CUDA matmul and a 16-package build on L4T 36.5.2. Keep the robot-tested external source unchanged.
- Jetson container startup uses NVIDIA runtime, explicit devices, stable `/dev/ttyMotor` and `/dev/ttyLidar` aliases, and host networking for ROS discovery. Never start motor or autonomous motion during unattended validation.
- `scripts/build_ws.sh` defaults to Laptop outputs; the Jetson run script sets `AFTR_BUILD_TARGET=jetson` and `AFTR_INSTALL_BASE=install_jetson`. Do not source or reuse an overlay from the other architecture.
- Keep common dependencies architecture-neutral where possible; isolate future Jetson dependencies.
- Hardware-dependent tests are not required for laptop CI. Do not run the default operator-system or motor bringup launch in laptop validation.
- Use `/models` for read-only model files and `/data` for writable paths, maps, and poses. Do not add personal `/home/...` defaults; keep ROS parameters available for overrides.
- Never share colcon build, install, or log outputs between amd64 and arm64 environments. Laptop Docker uses `build_laptop/`, `install_laptop/`, and `log_laptop/`; Jetson uses `build_jetson/`, `install_jetson/`, and `log_jetson/`. Keep these separate from host outputs.
- Update `docs/setup.md` when Docker scripts, workspace paths, runtime devices, or deployment steps change; keep only a short quick-start summary in `README.md`.
- Docker changes must preserve ROS runtime interfaces and hardware-sensitive behavior.
- The production GUI entry point is `aftr_gui.operator_gui:main`; preserve safety-stop rendering, release confirmation, snapshot handling, and mode-manager ownership of automatic base startup.
- Do not refactor `aftr_tracking`, `aftr_fall_detection`, `aftr_status_led`, or `aftr_path_manager` source outside explicit scope; path-only portability edits are the current exception.

## Documentation Rules

- Write `AGENTS.md` and code comments in English. Write the main `README.md` and user-facing documents under `docs/` primarily in Korean while retaining technical terms in English.
- Update documentation whenever package boundaries, ROS interfaces, architecture, workflow assumptions, or safety behavior change.
- Keep this `AGENTS.md` aligned with the current repository structure, package boundaries, and project-wide development rules.
- Keep `README.md` concise as the project entry point; place installation and Docker details in `docs/setup.md`.
- Keep README Quick Start to Docker image build, Container start, Workspace Build, and overlay source. Put test, lint, troubleshooting, and developer-only commands in `docs/maintenance.md`.
- Describe Linux amd64 with Docker as the intended Laptop environment; identify Ubuntu 26 LTS amd64 separately as the verified Phase 1 host. Avoid volatile Test counts in README.
- Put long explanations, design rationale, and refactoring guidance in `docs/`.
- Follow the existing source conventions and review ROS interface and hardware behavior whenever related code changes.

## Copyright and Source Comments

- New first-party source files use `Copyright (c) 2026 Autonomous Following and Towing Robot Contributors` in the native comment style, after any required shebang or XML declaration.
- Do not add personal author tags to source files. Team responsibilities are documented in `README.md`.
- Preserve inherited third-party copyright notices and licenses. Do not modify copyright or license headers in external sibling repositories.
- Write source comments and docstrings in English. Keep README and user-facing documentation Korean-first with English technical terms.
- Explain intent, constraints, safety behavior, hardware assumptions, ROS interface assumptions, or non-obvious logic. Do not restate self-explanatory code or add comments merely to increase coverage.
- Keep safety-critical and hardware-specific comments clear and explicit. TODO comments must describe the intended follow-up action. Avoid decorative section banners.
- Comment-only cleanup must preserve executable behavior, ROS interfaces, and hardware semantics.
- Keep public and non-trivial Python docstrings concise and practical; do not force a full Args/Returns/Raises template on every helper.
- Use short Doxygen comments for public C++ interfaces when useful and ordinary `//` comments for implementation details.

## Safety and Interface Stability

- Treat motor commands, serial communication, controller activation, emergency stop behavior, LiDAR handling, and Nav2 safety limits as hardware-sensitive.
- For hardware-affecting changes, explain the risk and prefer no-hardware or dry-run validation before real robot testing.
- Package identity uses `aftr_*`, but ROS runtime interfaces are separate. Do not rename topics, services, actions, frames, links, joints, parameters, node names, executables, plugin IDs, hardware identifiers, CSV formats, maps, or saved pose formats without an explicit migration plan.
- Preserve hardware communication and robot safety behavior.

## Commit Guidance

- Use Conventional Commits for commit messages.
