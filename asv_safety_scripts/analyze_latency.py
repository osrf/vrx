#!/usr/bin/env python3
"""Compute transparent repeated-run latency statistics from raw CSV data."""
import argparse
import csv
from pathlib import Path
import statistics


def percentile(values, p):
    ordered = sorted(values)
    position = (len(ordered) - 1) * p
    low, high = int(position), min(int(position) + 1, len(ordered) - 1)
    return ordered[low] + (ordered[high] - ordered[low]) * (position - low)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('csv_file', type=Path)
    args = parser.parse_args()
    with args.csv_file.open(newline='') as input_file:
        rows = list(csv.DictReader(input_file))
    latencies = [float(row['t_zero_wall_s']) for row in rows if row.get('t_zero_wall_s')]
    if not latencies:
        raise SystemExit('No uncensored t_zero_wall_s samples found.')
    print(f'runs={len(latencies)}')
    print(f'mean_s={statistics.mean(latencies):.6f}')
    print(f'median_s={statistics.median(latencies):.6f}')
    print(f'p95_s={percentile(latencies, 0.95):.6f}')
    print(f'p99_s={percentile(latencies, 0.99):.6f}')
    print(f'min_s={min(latencies):.6f}')
    print(f'max_s={max(latencies):.6f}')
    impulses = [float(row['i_stale_n_s']) for row in rows if row.get('i_stale_n_s')]
    if impulses:
        print(f'mean_i_stale_n_s={statistics.mean(impulses):.6f}')
        print(f'max_i_stale_n_s={max(impulses):.6f}')


if __name__ == '__main__':
    main()
