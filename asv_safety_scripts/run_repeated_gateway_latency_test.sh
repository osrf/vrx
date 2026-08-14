#!/usr/bin/env bash
# Repeated gateway-path latency test. VRX, MAVROS, and safety_gateway must
# already be running on the same native x86_64 host/container.
set -euo pipefail

RUNS=50
FREEZE_AFTER_S=3.0
SETTLE_AFTER_FREEZE_S=0.6
RESULTS_DIR=results/gateway_repeated
while [[ $# -gt 0 ]]; do
  case "$1" in
    --runs) RUNS="$2"; shift 2 ;;
    --results-dir) RESULTS_DIR="$2"; shift 2 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done

MONITOR=asv_safety_tools/build/gz_setpoint_monitor
if [[ ! -x "$MONITOR" ]]; then
  cmake -S asv_safety_tools -B asv_safety_tools/build
  cmake --build asv_safety_tools/build
fi
mkdir -p "$RESULTS_DIR"
METRICS="$RESULTS_DIR/metrics.csv"
rm -f "$METRICS"

cleanup_children() {
  [[ -n "${producer_pid:-}" ]] && kill "$producer_pid" 2>/dev/null || true
  [[ -n "${monitor_pid:-}" ]] && kill "$monitor_pid" 2>/dev/null || true
  [[ -n "${recorder_pid:-}" ]] && kill "$recorder_pid" 2>/dev/null || true
}
trap cleanup_children EXIT

for run in $(seq 1 "$RUNS"); do
  echo "Gateway trial $run/$RUNS"
  # On the first cold boot this may legitimately report that no fault is
  # latched. On all later trials it releases LATCHED_FAULT to UNARMED_WAIT.
  ros2 service call /asv_actuator_safety_gateway/reset_fault std_srvs/srv/Trigger '{}' >/dev/null 2>&1 || true
  trace="$RESULTS_DIR/run_${run}_setpoints.csv"
  accepted="$RESULTS_DIR/run_${run}_accepted.csv"
  "$MONITOR" > "$trace" 2> "$RESULTS_DIR/run_${run}_monitor.log" & monitor_pid=$!
  python3 asv_safety_scripts/record_accepted_commands.py --output "$accepted" & recorder_pid=$!
  sleep 0.5
  sleep 0.5
  ros2 run asv_actuator_safety_gateway mavlink_hil_producer \
    --freeze-after "$FREEZE_AFTER_S" > "$RESULTS_DIR/run_${run}_producer.log" 2>&1 & producer_pid=$!

  # 1.1 s is the producer's safe unarmed warmup. The remaining interval gives
  # a stable armed command period before the actuator stream freezes.
  sleep "$(python3 -c "print(${FREEZE_AFTER_S} + ${SETTLE_AFTER_FREEZE_S})")"
  kill "$producer_pid" 2>/dev/null || true; wait "$producer_pid" 2>/dev/null || true; unset producer_pid
  end_time=$(python3 -c 'import time; print(f"{time.monotonic():.9f}")')
  kill "$recorder_pid" 2>/dev/null || true; wait "$recorder_pid" 2>/dev/null || true; unset recorder_pid
  kill "$monitor_pid" 2>/dev/null || true; wait "$monitor_pid" 2>/dev/null || true; unset monitor_pid
  last_valid=$(tail -n 1 "$accepted" | cut -d, -f1)
  if [[ -z "$last_valid" ]]; then
    echo "No accepted command recorded for trial $run" >&2
    exit 1
  fi
  python3 asv_safety_scripts/extract_run_metrics.py "$trace" \
    --last-valid-command-steady-s "$last_valid" \
    --observation-end-steady-s "$end_time" --run-id "$run" --output "$METRICS"
done

python3 asv_safety_scripts/analyze_latency.py "$METRICS"
