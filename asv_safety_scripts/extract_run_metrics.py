#!/usr/bin/env python3
"""Derive one transparent A/B metric row from a GZ Transport setpoint trace.

The input is stdout from ``gz_setpoint_monitor``.  It deliberately measures
the plugin target setpoint, not a claimed propeller-speed sensor.  All times
are local CLOCK_MONOTONIC seconds from the same host.
"""
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


RHO_KG_M3 = 1000.0
THRUST_COEFFICIENT = 0.004422
PROPELLER_DIAMETER_M = 0.2


def force_from_omega(omega_rad_s: float) -> float:
    """Invert the pinned VRX thrust model for an angular-velocity setpoint."""
    magnitude = RHO_KG_M3 * THRUST_COEFFICIENT * PROPELLER_DIAMETER_M ** 4
    return math.copysign(magnitude * omega_rad_s ** 2, omega_rad_s * THRUST_COEFFICIENT)


def load_samples(path: Path) -> dict[str, list[tuple[float, float]]]:
    samples: dict[str, list[tuple[float, float]]] = {"left": [], "right": []}
    with path.open(newline="") as input_file:
        for row in csv.reader(input_file):
            if len(row) != 3 or row[1] not in samples:
                continue
            samples[row[1]].append((float(row[0]), float(row[2])))
    for stream in samples.values():
        stream.sort()
    if not all(samples.values()):
        raise ValueError("Trace must contain left and right setpoint samples.")
    return samples


def first_zero_after(samples: list[tuple[float, float]], start: float, epsilon: float) -> float | None:
    saw_nonzero = False
    for timestamp, omega in samples:
        if timestamp < start:
            continue
        if abs(omega) > epsilon:
            saw_nonzero = True
        elif saw_nonzero:
            return timestamp
    return None


def stale_impulse(samples: list[tuple[float, float]], start: float, end: float) -> float:
    """Zero-order-hold integral of absolute thrust over [start, end]."""
    current = 0.0
    for timestamp, omega in samples:
        if timestamp <= start:
            current = omega
        else:
            break
    previous = start
    impulse = 0.0
    for timestamp, omega in samples:
        if timestamp <= start:
            continue
        if timestamp >= end:
            break
        impulse += abs(force_from_omega(current)) * (timestamp - previous)
        current, previous = omega, timestamp
    impulse += abs(force_from_omega(current)) * (end - previous)
    return impulse


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("trace", type=Path)
    parser.add_argument("--last-valid-command-steady-s", required=True, type=float)
    parser.add_argument("--observation-end-steady-s", required=True, type=float)
    parser.add_argument("--run-id", default="1")
    parser.add_argument("--zero-epsilon-rad-s", type=float, default=1e-6)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.observation_end_steady_s <= args.last_valid_command_steady_s:
        parser.error("observation end must follow the last accepted command")

    samples = load_samples(args.trace)
    zero_times = [
        first_zero_after(samples[side], args.last_valid_command_steady_s, args.zero_epsilon_rad_s)
        for side in ("left", "right")
    ]
    t_zero = max(zero_times) if all(zero_times) else None
    impulse = sum(
        stale_impulse(stream, args.last_valid_command_steady_s, args.observation_end_steady_s)
        for stream in samples.values()
    )
    row = {
        "run_id": args.run_id,
        "last_valid_command_steady_s": f"{args.last_valid_command_steady_s:.9f}",
        "observation_end_steady_s": f"{args.observation_end_steady_s:.9f}",
        "t_zero_wall_s": "" if t_zero is None else f"{t_zero - args.last_valid_command_steady_s:.9f}",
        "right_censored": "true" if t_zero is None else "false",
        "i_stale_n_s": f"{impulse:.9f}",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    write_header = not args.output.exists() or args.output.stat().st_size == 0
    with args.output.open("a", newline="") as output_file:
        writer = csv.DictWriter(output_file, fieldnames=list(row))
        if write_header:
            writer.writeheader()
        writer.writerow(row)
    print(", ".join(f"{key}={value}" for key, value in row.items()))


if __name__ == "__main__":
    main()
