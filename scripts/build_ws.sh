#!/usr/bin/env bash
# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_root="$(cd "$repo_root/../.." && pwd)"
cd "$workspace_root"

build_target="${AFTR_BUILD_TARGET:-laptop}"
if [[ "$build_target" != laptop && "$build_target" != jetson ]]; then
  echo "AFTR_BUILD_TARGET must be laptop or jetson." >&2
  exit 2
fi

if [[ "${ROS_DISTRO:-}" != humble ]]; then
  echo "Source ROS 2 Humble before building this workspace." >&2
  exit 1
fi

colcon --log-base "log_$build_target" build --build-base "build_$build_target" --install-base "install_$build_target" --symlink-install "$@"

compile_commands_source="$workspace_root/build_$build_target/aftr_hardware/compile_commands.json"
if [[ -f "$compile_commands_source" ]]; then
  ln -sfn "../../build_$build_target/aftr_hardware/compile_commands.json" \
    "$repo_root/compile_commands.json"
fi
