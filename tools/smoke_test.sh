#!/usr/bin/env bash
# Headless smoke test for the vrx4 branch, in two comparable rendering backends.
#
#   smoke_test.sh xvfb   # GLX rendering under a virtual X server (Xvfb), CPU GL
#   smoke_test.sh egl    # EGL headless rendering (gz sim --headless-rendering)
#
# Both launch the simulation headless, then confirm /clock is advancing AND the
# observation camera is producing frames, reporting the achieved FPS. Run inside
# a built workspace with ROS 2 Lyrical + vendored Gazebo Jetty already sourced.
#
# Exit 0 on success, non-zero on any failure.
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BACKEND="${1:-xvfb}"
WORLD="${VRX_SMOKE_WORLD:-open_water.sdf}"
IMAGE_TOPIC="${VRX_SMOKE_IMAGE_TOPIC:-/observation_camera/image}"
# Require sustained rendering, not a single frame: a backend that renders once
# then stalls should fail. Deliberately well under the world's 30 Hz to stay
# non-flaky on software (llvmpipe) GL.
MIN_FRAMES="${VRX_SMOKE_MIN_FRAMES:-5}"
LOG="$(mktemp -t vrx_smoke_launch.XXXXXX.log)" || { echo "mktemp failed" >&2; exit 1; }

# Isolate this run's ROS graph so a stray /clock from another process on the
# runner (a leftover sim, a rosbag) can't satisfy the check. Launch and checker
# both inherit this, so they still talk to each other.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$(( $$ % 100 + 1 ))}"

# Both backends launch the SAME file (sim_smoke.launch.xml), differing only by
# headless_rendering -> the gz `--headless-rendering` flag. Everything else
# (process topology, render engine, -v1 verbosity, standalone bridge) is held
# equal so the camera-FPS numbers are an apples-to-apples GLX-vs-EGL compare.
declare -a WRAP LAUNCH
declare -a RENDERER_INFO
case "${BACKEND}" in
  xvfb)
    # Fake X display + Mesa software GL (llvmpipe), GLX rendering path.
    export LIBGL_ALWAYS_SOFTWARE=1
    export GALLIUM_DRIVER="${GALLIUM_DRIVER:-llvmpipe}"
    WRAP=(xvfb-run -a)
    RENDERER_INFO=(glxinfo -B)
    LAUNCH=(ros2 launch vrx_bringup sim_smoke.launch.xml
            world:="${WORLD}" headless_rendering:=false)
    ;;
  egl)
    # EGL headless rendering, no X server. gz selects an EGL device via
    # eglQueryDevicesEXT; on a GPU-less runner that resolves to Mesa's software
    # (llvmpipe) device. Do NOT set LIBGL_ALWAYS_SOFTWARE here: it conflicts with
    # explicit EGL device selection ("Not allowed to force software rendering").
    WRAP=()
    RENDERER_INFO=(eglinfo -B)
    LAUNCH=(ros2 launch vrx_bringup sim_smoke.launch.xml
            world:="${WORLD}" headless_rendering:=true)
    ;;
  *)
    echo "usage: smoke_test.sh <xvfb|egl>" >&2
    exit 2
    ;;
esac

# Record the llvmpipe thread setting so the FPS number has its threading context
# in the log (LP_NUM_THREADS controls llvmpipe's rasterizer worker threads).
echo "[smoke:${BACKEND}] LP_NUM_THREADS=${LP_NUM_THREADS:-<unset: llvmpipe auto>}"

# Record which GL renderer this backend actually bound (llvmpipe vs a silent
# softpipe/swrast fallback), so the FPS number in the log has a renderer next to
# it. Best-effort: never fail the smoke test just because the info tool is absent.
echo "[smoke:${BACKEND}] GL renderer (${RENDERER_INFO[*]}):"
if command -v "${RENDERER_INFO[0]}" >/dev/null 2>&1; then
  "${WRAP[@]}" "${RENDERER_INFO[@]}" 2>&1 \
    | grep -iE "vendor|renderer|version|device" | sed 's/^/  /' \
    || echo "  (renderer query returned nothing)"
else
  echo "  (${RENDERER_INFO[0]} not installed; skipping)"
fi

LAUNCH_PID=""
cleanup() {
  if [[ -n "${LAUNCH_PID}" ]]; then
    kill -- "-${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
  rm -f "${LOG}"
}
trap cleanup EXIT

echo "[smoke:${BACKEND}] launching simulation headless (world=${WORLD})..."
# setsid -> own session/process group so cleanup can kill the whole tree.
setsid "${WRAP[@]}" "${LAUNCH[@]}" >"${LOG}" 2>&1 &
LAUNCH_PID=$!

# Fail fast if the launcher could not even fork/exec.
if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
  echo "[smoke:${BACKEND}] FAILED: launch process did not start." >&2
  cat "${LOG}" >&2
  exit 1
fi

echo "[smoke:${BACKEND}] measuring /clock advance + camera FPS..."
# Capture rc inline (set -e is off) so no later edit can clobber $? before it.
rc=0
python3 "${SCRIPT_DIR}/check_render.py" \
  --backend "${BACKEND}" --image-topic "${IMAGE_TOPIC}" \
  --min-frames "${MIN_FRAMES}" || rc=$?

# Best-effort crash signal: ros2 launch keeps running if a child dies, so this
# only fires when the whole launch tree is already gone — but when it does, the
# failure was a crashed/failed launch, not a slow or paused sim.
if [[ ${rc} -ne 0 ]] && ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
  echo "[smoke:${BACKEND}] launch process exited before the check completed (crash)." >&2
fi

if [[ ${rc} -ne 0 ]]; then
  echo "[smoke:${BACKEND}] FAILED. Launch output follows:" >&2
  echo "----------------------------------------" >&2
  cat "${LOG}" >&2
  echo "----------------------------------------" >&2
  exit "${rc}"
fi

echo "[smoke:${BACKEND}] PASSED."
exit 0
