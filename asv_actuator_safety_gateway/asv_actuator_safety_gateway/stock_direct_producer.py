"""Baseline producer: directly writes VRX force topics and never sends a stop."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class StockDirectProducer(Node):
    def __init__(self) -> None:
        super().__init__('stock_direct_producer')
        self.declare_parameter('thrust_n', 200.0)
        self.declare_parameter('publish_for_s', 2.0)
        self._left = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self._right = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        self._started = self.get_clock().now()
        self._timer = self.create_timer(0.02, self.publish_until_freeze)

    def publish_until_freeze(self) -> None:
        elapsed = (self.get_clock().now() - self._started).nanoseconds / 1e9
        if elapsed >= self.get_parameter('publish_for_s').value:
            self._timer.cancel()
            self.get_logger().warn('Baseline command stream stopped; stock plugin retains its last setpoint.')
            return
        thrust = Float64(data=float(self.get_parameter('thrust_n').value))
        self._left.publish(thrust)
        self._right.publish(thrust)


def main() -> None:
    rclpy.init()
    node = StockDirectProducer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
