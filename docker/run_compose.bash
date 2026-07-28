#!/usr/bin/env bash
#
# Copyright (C) 2026 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
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

# Workspace root = three levels up from docker/ (…/<ws>/src/vrx/docker).
# Derived from the script's own location so the repo can live anywhere; the
# compose file falls back to a default if this isn't exported.
export VRX_WS
VRX_WS="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

# Let the container's (host-UID) process reach the host X server. Harmless to
# re-run; warn but don't abort if there's no X server (e.g. headless).
xhost +SI:localuser:"$(id -un)" >/dev/null 2>&1 \
  || echo "WARN: xhost failed (no X server?); GUI apps may not display." >&2

# Host UID/GID so files created in the mounted home keep your ownership.
# (`UID` is a bash readonly variable, hence the MY_ prefix the compose expects.)
export MY_UID MY_GID
MY_UID="$(id -u)"
MY_GID="$(id -g)"

# GPU selection: prefer the NVIDIA dGPU only when one is present AND Docker can
# actually use it, otherwise fall back to the Intel/AMD iGPU (base compose
# only). The NVIDIA overlay reserves an nvidia GPU device, so `docker compose`
# refuses to start unless BOTH the host driver (nvidia-smi) and Docker's nvidia
# runtime (from nvidia-container-toolkit) are present. Checking only the driver
# would let a host without the toolkit fail instead of falling back.
have_nvidia_runtime() {
  docker info --format '{{println .Runtimes}}' 2>/dev/null | grep -qw nvidia
}
compose_files=(-f "${SCRIPT_DIR}/compose.yaml")
if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L >/dev/null 2>&1 \
   && have_nvidia_runtime; then
  compose_files+=(-f "${SCRIPT_DIR}/compose.nvidia.yaml")
  echo "GPU: NVIDIA detected -> dGPU via PRIME offload." >&2
else
  echo "GPU: no usable NVIDIA (driver or container toolkit) -> Intel/AMD iGPU via /dev/dri." >&2
fi

exec docker compose "${compose_files[@]}" run --rm --remove-orphans dev "$@"
