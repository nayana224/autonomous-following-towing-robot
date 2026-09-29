#!/usr/bin/env bash
# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "$(uname -m)" != aarch64 ]]; then
  echo "Jetson image build requires an aarch64 host." >&2
  exit 2
fi
test -f "$repo_root/docker/jetson/Dockerfile"

docker build \
  --tag aftr-dev:humble-jetson \
  --file "$repo_root/docker/jetson/Dockerfile" \
  "$repo_root"
