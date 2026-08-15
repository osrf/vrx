"""Dependency-free safety state machine used by the ROS 2 gateway."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Optional, Sequence, Tuple


class SafetyState(str, Enum):
    UNARMED_WAIT = 'UNARMED_WAIT'
    ARMED_OUTPUT = 'ARMED_OUTPUT'
    LATCHED_FAULT = 'LATCHED_FAULT'


class FaultReason(str, Enum):
    CONNECTION_LOST = 'CONNECTION_LOST'
    COMMAND_TIMEOUT = 'COMMAND_TIMEOUT'
    INVALID_COMMAND = 'INVALID_COMMAND'


@dataclass(frozen=True)
class PolicyConfig:
    max_thrust_n: float = 250.0
    command_timeout_s: float = 0.2
    required_zero_commands: int = 10
    min_unarmed_s: float = 1.0
    zero_epsilon: float = 1e-6


class ActuatorSafetyPolicy:
    """Conservative gate around MAVROS actuator controls.

    ``now`` is always a local monotonic-clock value supplied by the caller.
    MAVROS owns MAVLink transport and connection detection; this policy adds
    command-stream freshness, VRX mapping, latching, and controlled recovery.
    """

    def __init__(self, config: PolicyConfig = PolicyConfig()) -> None:
        if config.command_timeout_s <= 0 or config.max_thrust_n <= 0:
            raise ValueError('command_timeout_s and max_thrust_n must be positive')
        if config.required_zero_commands <= 0 or config.min_unarmed_s < 0:
            raise ValueError('required_zero_commands must be positive and min_unarmed_s non-negative')
        self.config = config
        self.state = SafetyState.UNARMED_WAIT
        self.fault: Optional[FaultReason] = None
        self.connected = False
        self.armed = False
        self._unarmed_since: Optional[float] = None
        self._safe_zero_count = 0
        self._last_command_at: Optional[float] = None
        self._output = (0.0, 0.0)

    @property
    def output(self) -> Tuple[float, float]:
        return self._output

    @property
    def last_command_at(self) -> Optional[float]:
        return self._last_command_at

    def reset_fault(self, now: float) -> bool:
        """Explicitly clears only a latched fault; recovery still needs handshake."""
        if self.state != SafetyState.LATCHED_FAULT:
            return False
        self.state = SafetyState.UNARMED_WAIT
        self.fault = None
        self._safe_zero_count = 0
        self._last_command_at = None
        self._output = (0.0, 0.0)
        self._unarmed_since = now if self.connected and not self.armed else None
        return True

    def update_connection(self, connected: bool, armed: bool, now: float) -> None:
        self.connected, self.armed = connected, armed
        if not connected:
            self._output = (0.0, 0.0)
            if self.state == SafetyState.ARMED_OUTPUT:
                self._latch(FaultReason.CONNECTION_LOST)
            return

        if not armed:
            if self._unarmed_since is None:
                self._unarmed_since = now
            self._output = (0.0, 0.0)
            if self.state == SafetyState.ARMED_OUTPUT:
                self.state = SafetyState.UNARMED_WAIT
                self._safe_zero_count = 0
                self._last_command_at = None
            return

        # Arming is accepted only after the controlled unarmed/zero handshake.
        if self.state == SafetyState.UNARMED_WAIT and self._handshake_complete(now):
            self.state = SafetyState.ARMED_OUTPUT
            self._last_command_at = now
            self._unarmed_since = None
            self._safe_zero_count = 0
        elif self.state == SafetyState.UNARMED_WAIT:
            # An early arm attempt invalidates the partial handshake. Recovery
            # must observe a fresh continuous unarmed interval and zero stream.
            self._unarmed_since = None
            self._safe_zero_count = 0

    def accept_command(self, controls: Sequence[float], now: float) -> bool:
        if len(controls) < 2 or self.state == SafetyState.LATCHED_FAULT or not self.connected:
            return False
        left, right = controls[0], controls[1]
        if not all(math.isfinite(value) and -1.0 <= value <= 1.0 for value in (left, right)):
            self._latch(FaultReason.INVALID_COMMAND)
            return False

        if self.state == SafetyState.UNARMED_WAIT:
            if not self.armed and abs(left) <= self.config.zero_epsilon and abs(right) <= self.config.zero_epsilon:
                self._safe_zero_count += 1
            else:
                self._safe_zero_count = 0
            self._output = (0.0, 0.0)
            return False

        if self.state != SafetyState.ARMED_OUTPUT:
            return False
        self._last_command_at = now
        self._output = (left * self.config.max_thrust_n, right * self.config.max_thrust_n)
        return True

    def tick(self, now: float) -> None:
        if self.state != SafetyState.ARMED_OUTPUT:
            return
        if not self.connected:
            self._latch(FaultReason.CONNECTION_LOST)
        elif self._last_command_at is None or now - self._last_command_at > self.config.command_timeout_s:
            self._latch(FaultReason.COMMAND_TIMEOUT)

    def _handshake_complete(self, now: float) -> bool:
        return (
            self._unarmed_since is not None
            and now - self._unarmed_since >= self.config.min_unarmed_s
            and self._safe_zero_count >= self.config.required_zero_commands
        )

    def _latch(self, reason: FaultReason) -> None:
        self.state = SafetyState.LATCHED_FAULT
        self.fault = reason
        self._output = (0.0, 0.0)
