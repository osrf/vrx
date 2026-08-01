#!/usr/bin/env python3
"""Smoke check for the VRX simulation: sim is stepping AND rendering works.

Confirms two things over a measurement window:
  1. /clock advances (the physics/simulation is actually stepping).
  2. the observation camera publishes images, and reports the achieved frame
     rate (FPS).

Subscribes with BEST_EFFORT QoS so it receives from either reliable or
best-effort publishers. Exit 0 if the clock advanced and at least
--min-frames camera frames arrived; 1 on timeout or insufficient frames.
"""

import sys
import time
import argparse
from collections.abc import Callable

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image


class RenderWatcher(Node):
    def __init__(self, image_topic: str) -> None:
        super().__init__("vrx_smoke_render_watcher")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.latest_clock_ns: int | None = None
        self.image_count: int = 0
        self.create_subscription(Clock, "/clock", self._on_clock, qos)
        self.create_subscription(Image, image_topic, self._on_image, qos)

    def _on_clock(self, msg: Clock) -> None:
        self.latest_clock_ns = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec

    def _on_image(self, msg: Image) -> None:
        self.image_count += 1


def _spin_until(node: Node, predicate: Callable[[], bool], timeout: float) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return True
    return predicate()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--image-topic", default="/observation_camera/image")
    parser.add_argument(
        "--first-timeout",
        type=float,
        default=90.0,
        help="seconds to wait for the first /clock message (covers Fuel download "
        "and render-engine init)",
    )
    parser.add_argument(
        "--measure-secs",
        type=float,
        default=10.0,
        help="wall-clock window over which to count camera frames",
    )
    parser.add_argument("--min-frames", type=int, default=1)
    parser.add_argument(
        "--wait-only",
        action="store_true",
        help="exit 0 as soon as /clock is seen and skip the frame measurement; "
        "used to gate the runtime camera spawn in CI",
    )
    args = parser.parse_args()

    rclpy.init()
    node = None
    try:
        node = RenderWatcher(args.image_topic)

        if not _spin_until(
            node,
            lambda: node.latest_clock_ns is not None,
            args.first_timeout,
        ):
            node.get_logger().error(
                f"No /clock within {args.first_timeout:.0f}s — "
                "sim/bridge did not come up."
            )
            return 1

        if args.wait_only:
            node.get_logger().info("/clock detected; simulation is up.")
            return 0

        # Measure over a fixed wall-clock window: spin (to drain image callbacks)
        # for the whole window, counting frames.
        clock_at_start = node.latest_clock_ns
        node.image_count = 0
        t0 = time.monotonic()
        _spin_until(node, lambda: False, args.measure_secs)
        elapsed = time.monotonic() - t0

        clock_advanced = (
            node.latest_clock_ns is not None
            and clock_at_start is not None
            and node.latest_clock_ns > clock_at_start
        )
        frames = node.image_count
        fps = frames / elapsed if elapsed > 0 else 0.0

        node.get_logger().info(
            f"clock advancing: {clock_advanced} "
            f"({clock_at_start} -> {node.latest_clock_ns} ns)"
        )
        node.get_logger().info(
            f"CAMERA FPS: {fps:.2f} ({frames} frames on "
            f"{args.image_topic} over {elapsed:.1f}s)"
        )

        if not clock_advanced:
            node.get_logger().error("FAIL: /clock did not advance (paused?).")
            return 1
        if frames < args.min_frames:
            node.get_logger().error(
                f"FAIL: only {frames} camera frame(s) "
                f"(< {args.min_frames}) — rendering not producing images."
            )
            return 1

        node.get_logger().info("PASS: sim stepping and camera rendering.")
        return 0
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
