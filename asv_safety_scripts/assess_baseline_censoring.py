#!/usr/bin/env python3
"""Verify that a stock-path trace is right-censored rather than zeroed."""
from __future__ import annotations

import argparse
import csv
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('trace', type=Path)
    parser.add_argument('--fault-steady-s', required=True, type=float,
                        help='Local monotonic timestamp when the direct publisher stopped.')
    parser.add_argument('--minimum-observation-s', type=float, default=20.0)
    parser.add_argument('--epsilon-rad-s', type=float, default=1e-6)
    args = parser.parse_args()
    samples: dict[str, list[tuple[float, float]]] = {'left': [], 'right': []}
    with args.trace.open(newline='') as input_file:
        for row in csv.reader(input_file):
            if len(row) == 3 and row[1] in samples:
                samples[row[1]].append((float(row[0]), float(row[2])))
    if not all(samples.values()):
        raise SystemExit('Missing left or right setpoint samples.')

    for side, stream in samples.items():
        stream.sort()
        after_fault = [(timestamp, omega) for timestamp, omega in stream if timestamp >= args.fault_steady_s]
        if not after_fault:
            raise SystemExit(f'{side}: no samples after the supplied fault timestamp.')
        if not any(abs(omega) > args.epsilon_rad_s for _, omega in after_fault):
            raise SystemExit(f'{side}: no nonzero setpoint was observed.')
        end = after_fault[-1][0]
        if end - args.fault_steady_s < args.minimum_observation_s:
            raise SystemExit(
                f'{side}: only {end - args.fault_steady_s:.3f}s observed after publisher stop; '
                f'need {args.minimum_observation_s:.3f}s.')
        if any(abs(omega) <= args.epsilon_rad_s for _, omega in after_fault):
            raise SystemExit(f'{side}: setpoint reached zero; this is not a censored baseline trace.')
    print(f'PASS: both stock setpoints remained nonzero for > {args.minimum_observation_s:.3f}s after publication stopped.')


if __name__ == '__main__':
    main()
