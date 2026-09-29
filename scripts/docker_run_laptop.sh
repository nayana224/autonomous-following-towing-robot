#!/usr/bin/env bash
# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_root="$(cd "$repo_root/../.." && pwd)"
test -d "$workspace_root/src/laser_filters"
test -d "$workspace_root/src/serial-ros2"
test -d "$workspace_root/src/sllidar_ros2"

mode=auto
if [[ "${1:-}" == --headless ]]; then
  mode=headless
  shift
fi
if [[ "${1:-}" == --help ]]; then
  echo "Usage: $0 [--headless] [command [args...]]"
  exit 0
fi
if (( $# == 0 )); then
  set -- bash
fi

models_dir="${AFTR_MODELS_DIR:-$workspace_root/models}"
data_dir="${AFTR_DATA_DIR:-$workspace_root/data}"
if [[ "$models_dir" != /* || "$data_dir" != /* ]]; then
  echo "AFTR_MODELS_DIR and AFTR_DATA_DIR must be absolute paths." >&2
  exit 2
fi
mkdir -p "$models_dir" "$data_dir/paths" "$data_dir/maps" "$data_dir/poses"
if [[ ! -r "$models_dir" || ! -w "$data_dir" || ! -w "$data_dir/paths" || ! -w "$data_dir/maps" || ! -w "$data_dir/poses" ]]; then
  echo "Model directory must be readable and data directories must be writable by the host user." >&2
  exit 2
fi

host_uid="$(id -u)"
host_gid="$(id -g)"
docker_flags=(--rm --init --interactive --platform linux/amd64)
if [[ -t 0 && -t 1 ]]; then
  docker_flags+=(--tty)
fi
docker_flags+=(
  --user 0:0
  --env "AFTR_HOST_UID=$host_uid"
  --env "AFTR_HOST_GID=$host_gid"
  --env CUDA_VISIBLE_DEVICES=
  --env SDL_AUDIODRIVER=dummy
  --volume "$workspace_root:/workspace"
  --volume "$models_dir:/models:ro"
  --volume "$data_dir:/data:rw"
  --workdir /workspace
)

if [[ "$mode" == auto ]]; then
  display_number="$(printf '%s' "${DISPLAY:-}" | sed -nE 's/^:([0-9]+)(\.[0-9]+)?$/\1/p')"
  auth_file="${XAUTHORITY:-$HOME/.Xauthority}"
  if [[ -n "$display_number" && -S "/tmp/.X11-unix/X$display_number" && -f "$auth_file" && -r "$auth_file" ]]; then
    docker_flags+=(
      --env "DISPLAY=$DISPLAY"
      --env XAUTHORITY=/tmp/aftr.Xauthority
      --env QT_QPA_PLATFORM=xcb
      --env LIBGL_ALWAYS_SOFTWARE=1
      --env QT_X11_NO_MITSHM=1
      --env "XDG_RUNTIME_DIR=/tmp/aftr-runtime-$host_uid"
      --volume /tmp/.X11-unix:/tmp/.X11-unix:ro
      --volume "$auth_file:/tmp/aftr.Xauthority:ro"
    )
    echo "X11/XWayland GUI forwarding enabled (DISPLAY=$DISPLAY)." >&2
  else
    echo "No usable local X11/XWayland display; using headless mode." >&2
    mode=headless
  fi
fi
if [[ "$mode" == headless ]]; then
  docker_flags+=(--env QT_QPA_PLATFORM=offscreen)
fi

docker run "${docker_flags[@]}" aftr-dev:humble-cpu "$@"
