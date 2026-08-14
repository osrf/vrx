#!/usr/bin/env python3
"""Persist gateway command-acceptance timestamps for an A/B trial.

This recorder subscribes to the gateway's explicit test-instrumentation topic.
Both it and the Gazebo Transport monitor run on the same host and therefore
read the same kernel monotonic clock domain.
"""
from __future__ import annotations

import argparse
from pathlib import Path
import signal
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class Recorder(Node):
    def __init__(self, output: Path, topic: str) -> None:
        super().__init__('accepted_command_recorder')
        output.parent.mkdir(parents=True, exist_ok=True)
        self._file = output.open('w', encoding='utf-8')
        self.create_subscription(Float64, topic, self._callback, 50)

    def _callback(self, message: Float64) -> None:
        # message.data is stamped at the gateway callback boundary. The local
        # recorder timestamp is retained only as an audit trail.
        self._file.write(f'{message.data:.9f},{time.monotonic():.9f}\n')
        self._file.flush()

    def close(self) -> None:
        self._file.close()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument(
        '--topic', default='/asv_actuator_safety_gateway/last_accepted_command_steady_s')
    args = parser.parse_args()
    rclpy.init()
    node = Recorder(args.output, args.topic)
    shutdown = False

    def stop(_signum: int, _frame: object) -> None:
        nonlocal shutdown
        shutdown = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        while rclpy.ok() and not shutdown:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
