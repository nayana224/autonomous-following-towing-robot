#!/usr/bin/env bash
# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
set -eo pipefail

uid="${AFTR_HOST_UID:?AFTR_HOST_UID is required}"
gid="${AFTR_HOST_GID:?AFTR_HOST_GID is required}"
if [[ ! "$uid" =~ ^[0-9]+$ ]] || [[ ! "$gid" =~ ^[0-9]+$ ]]; then
  echo "AFTR_HOST_UID and AFTR_HOST_GID must be numeric." >&2
  exit 1
fi

if [[ "$uid" == 0 ]]; then
  username=root
  home=/root
else
  if ! getent group "$gid" >/dev/null; then
    groupadd --gid "$gid" "aftr-$gid"
  fi
  if existing_user="$(getent passwd "$uid")"; then
    username="${existing_user%%:*}"
  else
    username=aftr
    if getent passwd "$username" >/dev/null; then
      username="aftr-$uid"
    fi
    useradd --uid "$uid" --gid "$gid" --home-dir "/home/aftr-$uid" \
      --no-create-home --shell /bin/bash "$username"
  fi
  home="/home/aftr-$uid"
  mkdir -p "$home"
  chown "$uid:$gid" "$home"
  chmod 700 "$home"
  if [[ -n "${AFTR_DEVICE_GIDS:-}" ]]; then
    for device_gid in ${AFTR_DEVICE_GIDS}; do
      if [[ ! "$device_gid" =~ ^[0-9]+$ ]]; then
        echo "AFTR_DEVICE_GIDS must contain numeric group IDs." >&2
        exit 1
      fi
      if [[ "$device_gid" != "$gid" ]]; then
        if ! getent group "$device_gid" >/dev/null; then
          groupadd --gid "$device_gid" "aftr-device-$device_gid"
        fi
        device_group="$(getent group "$device_gid" | cut -d: -f1)"
        usermod --append --groups "$device_group" "$username"
      fi
    done
  fi
fi

export HOME="$home" USER="$username" LOGNAME="$username"
if [[ -n "${XDG_RUNTIME_DIR:-}" ]]; then
  mkdir -p "$XDG_RUNTIME_DIR"
  chown "$uid:$gid" "$XDG_RUNTIME_DIR"
  chmod 700 "$XDG_RUNTIME_DIR"
fi

source /opt/ros/humble/setup.bash
install_base="${AFTR_INSTALL_BASE:-install_laptop}"
if [[ "$install_base" != install_laptop && "$install_base" != install_jetson ]]; then
  echo "AFTR_INSTALL_BASE must be install_laptop or install_jetson." >&2
  exit 1
fi
if [[ -f "/workspace/$install_base/setup.bash" ]]; then
  source "/workspace/$install_base/setup.bash"
fi

if [[ "$uid" == 0 ]]; then
  exec "$@"
fi
exec setpriv --reuid "$uid" --regid "$gid" --init-groups -- "$@"
