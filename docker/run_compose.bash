#!/usr/bin/env bash
#
# Minimal launcher for the VRX 4 dev container via Docker Compose
# (ROS 2 Lyrical + Gazebo Jetty). Mirrors rocker: interactive shell, home mount,
# GPU, X11, joystick. Service definition lives in docker/compose.yaml.
#
# Usage:
#   ./docker/run_compose.bash             # interactive bash in the container
#   ./docker/run_compose.bash <cmd...>    # run a command instead of bash
#
set -euo pipefail

# Resolve compose.yaml next to this script, so it works from any directory.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Let the container's (host-UID) process reach the host X server. Harmless to
# re-run; warn but don't abort if there's no X server (e.g. headless).
xhost +SI:localuser:"$(id -un)" >/dev/null 2>&1 \
  || echo "WARN: xhost failed (no X server?); GUI apps may not display." >&2

# Host UID/GID so files created in the mounted home keep your ownership.
# (`UID` is a bash readonly variable, hence the MY_ prefix the compose expects.)
export MY_UID MY_GID
MY_UID="$(id -u)"
MY_GID="$(id -g)"

exec docker compose -f "${SCRIPT_DIR}/compose.yaml" run --rm --remove-orphans dev "$@"
