"""ROS 2 adapter around :mod:`safety_policy`.

MAVROS is deliberately upstream of this node.  This package neither opens a
UDP socket nor parses MAVLink packets.
"""
from __future__ import annotations

import time
from threading import Lock

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from mavros_msgs.msg import HilActuatorControls, State
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

from .safety_policy import ActuatorSafetyPolicy, PolicyConfig, SafetyState


class ActuatorSafetyGateway(Node):
    def __init__(self) -> None:
        super().__init__('asv_actuator_safety_gateway')
        self.declare_parameter('max_thrust_n', 250.0)
        self.declare_parameter('command_timeout_s', 0.2)
        self.declare_parameter('watchdog_period_s', 0.01)
        self.declare_parameter('required_zero_commands', 10)
        self.declare_parameter('min_unarmed_s', 1.0)
        self.declare_parameter('mavros_state_topic', '/mavros/state')
        self.declare_parameter('actuator_controls_topic', '/mavros/hil/actuator_controls')
        self.declare_parameter('left_thrust_topic', '/wamv/thrusters/left/thrust')
        self.declare_parameter('right_thrust_topic', '/wamv/thrusters/right/thrust')

        config = PolicyConfig(
            max_thrust_n=float(self.get_parameter('max_thrust_n').value),
            command_timeout_s=float(self.get_parameter('command_timeout_s').value),
            required_zero_commands=int(self.get_parameter('required_zero_commands').value),
            min_unarmed_s=float(self.get_parameter('min_unarmed_s').value),
        )
        self.policy = ActuatorSafetyPolicy(config)
        self._lock = Lock()
        self._left_pub = self.create_publisher(Float64, self.get_parameter('left_thrust_topic').value, 10)
        self._right_pub = self.create_publisher(Float64, self.get_parameter('right_thrust_topic').value, 10)
        # Test instrumentation only: this exposes the gateway's local
        # CLOCK_MONOTONIC receipt timestamp. It is deliberately separate from
        # MAVLink's remote time_usec and makes the wall-clock KPI auditable.
        self._accepted_command_time_pub = self.create_publisher(
            Float64, '~/last_accepted_command_steady_s', 10)
        self._diagnostics_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_subscription(State, self.get_parameter('mavros_state_topic').value, self._state_callback, 20)
        self.create_subscription(
            HilActuatorControls,
            self.get_parameter('actuator_controls_topic').value,
            self._controls_callback,
            50,
        )
        self.create_service(Trigger, '~/reset_fault', self._reset_callback)
        self.create_timer(float(self.get_parameter('watchdog_period_s').value), self._watchdog_callback)
        self._publish_output_and_diagnostics('startup')
        self.get_logger().info('Safety gateway ready; outputs are gated until unarmed/zero/armed handshake completes.')

    @staticmethod
    def _steady_now() -> float:
        return time.monotonic()

    def _state_callback(self, msg: State) -> None:
        with self._lock:
            self.policy.update_connection(msg.connected, msg.armed, self._steady_now())
            self._publish_output_and_diagnostics('mavros_state')

    def _controls_callback(self, msg: HilActuatorControls) -> None:
        # Deliberately use callback receipt time, never remote MAVLink time_usec.
        with self._lock:
            receipt_time = self._steady_now()
            accepted = self.policy.accept_command(msg.controls, receipt_time)
            if accepted:
                self._accepted_command_time_pub.publish(Float64(data=receipt_time))
            self._publish_output_and_diagnostics('accepted_command' if accepted else 'gated_command')

    def _watchdog_callback(self) -> None:
        with self._lock:
            previous = self.policy.state
            self.policy.tick(self._steady_now())
            if previous != self.policy.state and self.policy.state == SafetyState.LATCHED_FAULT:
                self.get_logger().error(f'Safety fault latched: {self.policy.fault.value}')
            self._publish_output_and_diagnostics('watchdog')

    def _reset_callback(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        with self._lock:
            reset = self.policy.reset_fault(self._steady_now())
            response.success = reset
            response.message = (
                'Fault cleared: awaiting unarmed heartbeat, zero controls, then arming.'
                if reset else 'No latched fault to reset.'
            )
            self._publish_output_and_diagnostics('explicit_reset')
            return response

    def _publish_output_and_diagnostics(self, source: str) -> None:
        left, right = self.policy.output
        self._left_pub.publish(Float64(data=left))
        self._right_pub.publish(Float64(data=right))
        level = DiagnosticStatus.ERROR if self.policy.state == SafetyState.LATCHED_FAULT else DiagnosticStatus.OK
        status = DiagnosticStatus(
            level=level,
            name='asv_actuator_safety_gateway',
            message=self.policy.fault.value if self.policy.fault else self.policy.state.value,
            hardware_id='vrx_wamv_actuator_boundary',
            values=[
                KeyValue(key='state', value=self.policy.state.value),
                KeyValue(key='connected', value=str(self.policy.connected)),
                KeyValue(key='armed', value=str(self.policy.armed)),
                KeyValue(key='source', value=source),
                KeyValue(key='left_commanded_thrust_n', value=f'{left:.6f}'),
                KeyValue(key='right_commanded_thrust_n', value=f'{right:.6f}'),
            ],
        )
        self._diagnostics_pub.publish(DiagnosticArray(status=[status]))


def main() -> None:
    rclpy.init()
    node = ActuatorSafetyGateway()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
