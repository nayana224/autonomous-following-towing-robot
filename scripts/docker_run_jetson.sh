#!/usr/bin/env bash
# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_root="$(cd "$repo_root/../.." && pwd)"
for sibling in laser_filters serial-ros2 sllidar_ros2; do
  test -d "$workspace_root/src/$sibling"
done
if [[ "$(uname -m)" != aarch64 ]]; then
  echo "Jetson container requires an aarch64 host." >&2
  exit 2
fi

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
docker_flags=(
  --rm --init --interactive
  --runtime nvidia --gpus all
  --network host
  --user 0:0
  --env "AFTR_HOST_UID=$host_uid"
  --env "AFTR_HOST_GID=$host_gid"
  --env AFTR_INSTALL_BASE=install_jetson
  --env AFTR_BUILD_TARGET=jetson
  --env NVIDIA_VISIBLE_DEVICES=all
  --env NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,video,display
  --env JETSON_MODEL_NAME=JETSON_ORIN_NANO
  --volume "$workspace_root:/workspace"
  --volume "$models_dir:/models:ro"
  --volume "$data_dir:/data:rw"
  --workdir /workspace
)
if [[ -t 0 && -t 1 ]]; then
  docker_flags+=(--tty)
fi

device_gids=()
add_device() {
  local source="$1" target="${2:-$1}" device_gid
  if [[ -c "$source" ]]; then
    docker_flags+=(--device "$source:$target")
    device_gid="$(stat -L -c %g "$source")"
    if [[ "$device_gid" != 0 && "$device_gid" != "$host_gid" && ! " ${device_gids[*]} " =~ " $device_gid " ]]; then
      device_gids+=("$device_gid")
      docker_flags+=(--group-add "$device_gid")
    fi
  fi
}

# Preserve the stable aliases used by the existing ROS configuration.
for alias in ttyMotor ttyLidar; do
  if [[ -e "/dev/$alias" ]]; then
    add_device "$(readlink -f "/dev/$alias")" "/dev/$alias"
  else
    echo "Optional /dev/$alias is absent; check the host udev rules before hardware launch." >&2
  fi
done
for device in /dev/video* /dev/gpiochip* /dev/snd/*; do
  add_device "$device"
done
# NVIDIA runtime mounts GPU nodes; preserve their host group permissions.
for device in /dev/nvmap /dev/nvhost-gpu /dev/nvgpu/igpu0/{as,channel,ctrl,nvsched,power,tsg} /dev/dri/render*; do
  if [[ -c "$device" ]]; then
    device_gid="$(stat -L -c %g "$device")"
    if [[ "$device_gid" != 0 && "$device_gid" != "$host_gid" && ! " ${device_gids[*]} " =~ " $device_gid " ]]; then
      device_gids+=("$device_gid")
      docker_flags+=(--group-add "$device_gid")
    fi
  fi
done
if [[ -d /dev/bus/usb ]]; then
  docker_flags+=(
    --volume /dev/bus/usb:/dev/bus/usb:rw
    --device-cgroup-rule 'c 189:* rwm'
  )
  for device in /dev/bus/usb/*/*; do
    if [[ -c "$device" ]]; then
      device_gid="$(stat -L -c %g "$device")"
      if [[ "$device_gid" != 0 && "$device_gid" != "$host_gid" && ! " ${device_gids[*]} " =~ " $device_gid " ]]; then
        device_gids+=("$device_gid")
        docker_flags+=(--group-add "$device_gid")
      fi
    fi
  done
fi
docker_flags+=(--env "AFTR_DEVICE_GIDS=${device_gids[*]}")

if [[ "$mode" == auto ]]; then
  display_number="$(printf '%s' "${DISPLAY:-}" | sed -nE 's/^:([0-9]+)(\.[0-9]+)?$/\1/p')"
  auth_file="${XAUTHORITY:-$HOME/.Xauthority}"
  if [[ -n "$display_number" && -S "/tmp/.X11-unix/X$display_number" && -f "$auth_file" && -r "$auth_file" ]]; then
    docker_flags+=(
      --env "DISPLAY=$DISPLAY"
      --env XAUTHORITY=/tmp/aftr.Xauthority
      --env QT_QPA_PLATFORM=xcb
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
  docker_flags+=(--env QT_QPA_PLATFORM=offscreen --env SDL_AUDIODRIVER=dummy)
fi

docker run "${docker_flags[@]}" aftr-dev:humble-jetson "$@"
