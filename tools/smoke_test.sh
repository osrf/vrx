#!/usr/bin/env bash
# Headless smoke test for vrx4: GLX rendering under a virtual X server (Xvfb)
# with Mesa software GL (llvmpipe). It launches the real production launch
# headless, then spawns a test-only observation camera into the running world
# (the camera is deliberately NOT part of the production world), bridges its
# image to ROS, and confirms /clock is advancing AND the camera is producing
# frames, reporting the achieved FPS. Run inside a built workspace with ROS 2
# Lyrical + vendored Gazebo Jetty already sourced.
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD="${VRX_SMOKE_WORLD:-open_water.sdf}"
# World *name* (the <world name="..."> attribute), needed to address the
# entity-spawn service /world/<name>/create. open_water.sdf is "default".
WORLD_NAME="${VRX_SMOKE_WORLD_NAME:-default}"
CAMERA_SDF="${VRX_SMOKE_CAMERA_SDF:-${SCRIPT_DIR}/observation_camera.sdf}"
IMAGE_TOPIC="${VRX_SMOKE_IMAGE_TOPIC:-/observation_camera/image}"
MIN_FRAMES="${VRX_SMOKE_MIN_FRAMES:-5}"
LOG="$(mktemp -t vrx_smoke_launch.XXXXXX.log)" || { echo "mktemp failed" >&2; exit 1; }

# Isolate this run's ROS graph so a stray /clock from another process on the
# runner (a leftover sim, a rosbag) can't satisfy the check. Launch and checker
# both inherit this, so they still talk to each other.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$(( $$ % 100 + 1 ))}"

# Fake X display + Mesa software GL (llvmpipe), GLX rendering path. Exercises the
# project's real composable launch (gazebo_gui:=false for headless).
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER="${GALLIUM_DRIVER:-llvmpipe}"
LAUNCH=(ros2 launch vrx_bringup simulation.launch.xml
        gazebo_gui:=false world:="${WORLD}")

LAUNCH_PID=""
BRIDGE_PID=""
cleanup() {
  [[ -n "${BRIDGE_PID}" ]] && kill "${BRIDGE_PID}" 2>/dev/null || true
  if [[ -n "${LAUNCH_PID}" ]]; then
    kill -- "-${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
  rm -f "${LOG}"
}
trap cleanup EXIT

echo "[smoke] launching simulation headless (world=${WORLD})..."
# setsid -> own session/process group so cleanup can kill the whole tree.
setsid xvfb-run -a "${LAUNCH[@]}" >"${LOG}" 2>&1 &
LAUNCH_PID=$!

# Fail fast if the launcher could not even fork/exec.
if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
  echo "[smoke] FAILED: launch process did not start." >&2
  cat "${LOG}" >&2
  exit 1
fi

# Wait for the sim to be up before injecting the camera: a /clock message means
# the world finished loading (Fuel download on first run) and is stepping, so
# the /world/<name>/create service is registered. Reuse the checker's proven
# rclpy /clock detection (--wait-only) instead of `ros2 topic echo`.
echo "[smoke] waiting for the simulation to come up (/clock)..."
if ! python3 "${SCRIPT_DIR}/check_render.py" --wait-only --first-timeout 180; then
  echo "[smoke] FAILED: sim/bridge did not come up (no /clock)." >&2
  cat "${LOG}" >&2
  exit 1
fi

# Spawn the test-only observation camera into the now-loaded world. Pose is
# given here (not in the model SDF) so placement — a test concern — lives with
# the test. The Sensors system already loaded by the world renders it. A short
# retry covers the race between /clock and the create service registering.
echo "[smoke] spawning observation camera into world '${WORLD_NAME}'..."
spawned=0
for _ in 1 2 3 4 5; do
  if ros2 run ros_gz_sim create \
       -world "${WORLD_NAME}" \
       -file "${CAMERA_SDF}" \
       -name observation_camera \
       -x 8 -y 8 -z 10 -R 0 -P 0.5 -Y -2.36 >>"${LOG}" 2>&1; then
    spawned=1
    break
  fi
  sleep 2
done
if [[ ${spawned} -ne 1 ]]; then
  echo "[smoke] FAILED: could not spawn the observation camera." >&2
  cat "${LOG}" >&2
  exit 1
fi

# Bridge the camera image to ROS just for this test (the production bridge config
# stays free of CI-only topics).
echo "[smoke] bridging ${IMAGE_TOPIC} (test-only)..."
ros2 run ros_gz_bridge parameter_bridge \
  "${IMAGE_TOPIC}@sensor_msgs/msg/Image[gz.msgs.Image" >>"${LOG}" 2>&1 &
BRIDGE_PID=$!

echo "[smoke] measuring /clock advance + camera FPS..."
# Capture rc inline (set -e is off) so no later edit can clobber $? before it.
rc=0
python3 "${SCRIPT_DIR}/check_render.py" \
  --image-topic "${IMAGE_TOPIC}" \
  --min-frames "${MIN_FRAMES}" || rc=$?

# Best-effort crash signal: ros2 launch keeps running if a child dies, so this
# only fires when the whole launch tree is already gone — but when it does, the
# failure was a crashed/failed launch, not a slow or paused sim.
if [[ ${rc} -ne 0 ]] && ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
  echo "[smoke] launch process exited before the check completed (crash)." >&2
fi

if [[ ${rc} -ne 0 ]]; then
  echo "[smoke] FAILED. Launch output follows:" >&2
  echo "----------------------------------------" >&2
  cat "${LOG}" >&2
  echo "----------------------------------------" >&2
  exit "${rc}"
fi

echo "[smoke] PASSED."
exit 0
