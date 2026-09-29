#!/usr/bin/env bash
# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_root="$(cd "$repo_root/../.." && pwd)"
cd "$workspace_root"

if [[ "${ROS_DISTRO:-}" != humble ]]; then
  echo "Source ROS 2 Humble before building this workspace." >&2
  exit 1
fi

colcon --log-base log_laptop build --build-base build_laptop --install-base install_laptop --symlink-install "$@"
