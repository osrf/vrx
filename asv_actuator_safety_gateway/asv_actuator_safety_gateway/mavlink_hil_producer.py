"""Independent MAVLink test producer. It has no ROS dependency."""
from __future__ import annotations

import argparse
import threading
import time


def main() -> None:
    parser = argparse.ArgumentParser(description='Send MAVLink HIL actuator controls over UDP.')
    parser.add_argument('--endpoint', default='udpout:127.0.0.1:14560')
    parser.add_argument('--left', type=float, default=0.8)
    parser.add_argument('--right', type=float, default=0.8)
    parser.add_argument('--actuator-hz', type=float, default=50.0)
    parser.add_argument('--heartbeat-hz', type=float, default=1.0)
    parser.add_argument('--unarmed-warmup-s', type=float, default=1.1,
                        help='Unarmed zero-command handshake duration before arming.')
    parser.add_argument('--freeze-after', type=float, default=None,
                        help='Stop only the actuator stream after this many seconds; heartbeat continues.')
    args = parser.parse_args()
    if not (-1.0 <= args.left <= 1.0 and -1.0 <= args.right <= 1.0):
        parser.error('left/right must be in [-1, 1]')

    try:
        from pymavlink import mavutil
    except ImportError as error:
        raise SystemExit('pymavlink is required: pip install pymavlink') from error

    mav = mavutil.mavlink_connection(args.endpoint, source_system=42, source_component=191)
    stop = threading.Event()
    start = time.monotonic()
    armed_flag = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED

    def heartbeat_loop() -> None:
        period = 1.0 / args.heartbeat_hz
        while not stop.is_set():
            is_armed = time.monotonic() - start >= args.unarmed_warmup_s
            mav.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_SURFACE_BOAT,
                mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                armed_flag if is_armed else 0,
                0,
                mavutil.mavlink.MAV_STATE_ACTIVE,
            )
            stop.wait(period)

    heartbeat = threading.Thread(target=heartbeat_loop, daemon=True)
    heartbeat.start()
    period = 1.0 / args.actuator_hz
    try:
        while True:
            elapsed = time.monotonic() - start
            if elapsed < args.unarmed_warmup_s:
                controls = [0.0, 0.0] + [0.0] * 14
                mode = 0
            else:
                controls = [args.left, args.right] + [0.0] * 14
                mode = armed_flag
            if args.freeze_after is None or elapsed < args.freeze_after:
                mav.mav.hil_actuator_controls_send(
                    int(elapsed * 1_000_000), controls, mode, 0,
                )
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        heartbeat.join(timeout=1.0)
