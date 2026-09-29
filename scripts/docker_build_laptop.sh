#!/usr/bin/env bash
# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
test -f "$repo_root/docker/laptop/Dockerfile"

docker build \
  --platform linux/amd64 \
  --tag aftr-dev:humble-cpu \
  --file "$repo_root/docker/laptop/Dockerfile" \
  "$repo_root"
