#!/usr/bin/env python3
"""Smoke check for the VRX simulation: sim is stepping AND rendering works.

Confirms two things over a measurement window:
  1. /clock advances (the physics/simulation is actually stepping).
  2. the observation camera publishes images, and reports the achieved frame
     rate (FPS) — the headline number for comparing rendering backends
     (GLX-under-Xvfb vs EGL headless).

Subscribes with BEST_EFFORT QoS so it receives from either reliable or
best-effort publishers. Exit 0 if the clock advanced and at least
--min-frames camera frames arrived; 1 on timeout or insufficient frames;
2 if a /clock stamp could not be read.
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


def stamp_ns(msg: Clock) -> int:
    return msg.clock.sec * 1_000_000_000 + msg.clock.nanosec


class RenderWatcher(Node):
    def __init__(self, image_topic: str) -> None:
        super().__init__("vrx_smoke_render_watcher")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.first_clock_ns: int | None = None
        self.latest_clock_ns: int | None = None
        self.image_count: int = 0
        # First exception raised inside a callback, if any. Surfaced so a failure
        # reading a /clock stamp is reported as itself, not as a timeout.
        self.callback_error: Exception | None = None
        self.create_subscription(Clock, "/clock", self._on_clock, qos)
        self.create_subscription(Image, image_topic, self._on_image, qos)

    def _on_clock(self, msg: Clock) -> None:
        try:
            ns = stamp_ns(msg)
        except Exception as exc:  # noqa: BLE001 — re-raised via callback_error
            if self.callback_error is None:
                self.callback_error = exc
            return
        if self.first_clock_ns is None:
            self.first_clock_ns = ns
        self.latest_clock_ns = ns

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
    parser.add_argument("--backend", default="", help="label for the report line")
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
    args = parser.parse_args()

    rclpy.init()
    node = None
    try:
        node = RenderWatcher(args.image_topic)
        label = f"[{args.backend}] " if args.backend else ""

        if not _spin_until(
            node,
            lambda: node.first_clock_ns is not None or node.callback_error is not None,
            args.first_timeout,
        ):
            node.get_logger().error(
                f"{label}No /clock within {args.first_timeout:.0f}s — "
                "sim/bridge did not come up."
            )
            return 1
        if node.callback_error is not None:
            node.get_logger().error(
                f"{label}error reading a /clock stamp: {node.callback_error!r}"
            )
            return 2

        # Measure over a fixed wall-clock window.
        clock_at_start = node.latest_clock_ns
        node.image_count = 0
        t0 = time.monotonic()
        _spin_until(node, lambda: node.callback_error is not None, args.measure_secs)
        elapsed = time.monotonic() - t0

        if node.callback_error is not None:
            node.get_logger().error(
                f"{label}error reading a /clock stamp: {node.callback_error!r}"
            )
            return 2

        clock_advanced = (
            node.latest_clock_ns is not None
            and clock_at_start is not None
            and node.latest_clock_ns > clock_at_start
        )
        frames = node.image_count
        fps = frames / elapsed if elapsed > 0 else 0.0

        node.get_logger().info(
            f"{label}clock advancing: {clock_advanced} "
            f"({clock_at_start} -> {node.latest_clock_ns} ns)"
        )
        node.get_logger().info(
            f"{label}CAMERA FPS: {fps:.2f} ({frames} frames on "
            f"{args.image_topic} over {elapsed:.1f}s)"
        )

        if not clock_advanced:
            node.get_logger().error(f"{label}FAIL: /clock did not advance (paused?).")
            return 1
        if frames < args.min_frames:
            node.get_logger().error(
                f"{label}FAIL: only {frames} camera frame(s) "
                f"(< {args.min_frames}) — rendering not producing images."
            )
            return 1

        node.get_logger().info(f"{label}PASS: sim stepping and camera rendering.")
        return 0
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
