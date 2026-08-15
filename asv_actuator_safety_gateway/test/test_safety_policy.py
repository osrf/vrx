import unittest

from asv_actuator_safety_gateway.safety_policy import (
    ActuatorSafetyPolicy, FaultReason, PolicyConfig, SafetyState,
)


def ready_policy():
    policy = ActuatorSafetyPolicy(PolicyConfig(min_unarmed_s=1.0, required_zero_commands=3))
    policy.update_connection(True, False, 0.0)
    for time in (0.1, 0.2, 0.3):
        assert not policy.accept_command([0.0, 0.0], time)
    policy.update_connection(True, True, 1.0)
    assert policy.state == SafetyState.ARMED_OUTPUT
    return policy


class SafetyPolicyTests(unittest.TestCase):
    def test_command_maps_normalized_channels_to_newtons(self):
        policy = ready_policy()
        self.assertTrue(policy.accept_command([0.8, -0.4], 1.01))
        self.assertEqual(policy.output, (200.0, -100.0))


    def test_stale_actuator_stream_latches_even_if_connection_is_alive(self):
        policy = ready_policy()
        policy.accept_command([0.8, 0.8], 1.01)
        policy.tick(1.22)
        self.assertEqual(policy.state, SafetyState.LATCHED_FAULT)
        self.assertEqual(policy.fault, FaultReason.COMMAND_TIMEOUT)
        self.assertEqual(policy.output, (0.0, 0.0))


    def test_latched_fault_rejects_commands_until_explicit_reset_and_handshake(self):
        policy = ready_policy()
        policy.accept_command([0.8, 0.8], 1.01)
        policy.tick(1.22)
        self.assertFalse(policy.accept_command([0.8, 0.8], 1.23))
        self.assertTrue(policy.reset_fault(2.0))
        policy.update_connection(True, False, 2.0)
        for time in (2.2, 2.3, 2.4):
            policy.accept_command([0.0, 0.0], time)
        policy.update_connection(True, True, 3.1)
        self.assertEqual(policy.state, SafetyState.ARMED_OUTPUT)


    def test_connection_loss_is_latched_and_zeroed(self):
        policy = ready_policy()
        policy.accept_command([0.8, 0.8], 1.01)
        policy.update_connection(False, True, 1.05)
        self.assertEqual(policy.state, SafetyState.LATCHED_FAULT)
        self.assertEqual(policy.fault, FaultReason.CONNECTION_LOST)
        self.assertEqual(policy.output, (0.0, 0.0))

    def test_invalid_command_is_latched_and_zeroed(self):
        policy = ready_policy()
        self.assertFalse(policy.accept_command([float('nan'), 0.2], 1.01))
        self.assertEqual(policy.state, SafetyState.LATCHED_FAULT)
        self.assertEqual(policy.fault, FaultReason.INVALID_COMMAND)
        self.assertEqual(policy.output, (0.0, 0.0))

    def test_early_arm_invalidates_partial_handshake(self):
        policy = ActuatorSafetyPolicy(
            PolicyConfig(min_unarmed_s=1.0, required_zero_commands=3)
        )
        policy.update_connection(True, False, 0.0)
        policy.accept_command([0.0, 0.0], 0.2)
        policy.accept_command([0.0, 0.0], 0.3)

        policy.update_connection(True, True, 0.5)
        self.assertEqual(policy.state, SafetyState.UNARMED_WAIT)

        policy.update_connection(True, False, 0.6)
        for now in (0.7, 0.8, 0.9):
            policy.accept_command([0.0, 0.0], now)
        policy.update_connection(True, True, 1.5)
        self.assertEqual(policy.state, SafetyState.UNARMED_WAIT)

        policy.update_connection(True, False, 1.6)
        for now in (1.7, 1.8, 1.9):
            policy.accept_command([0.0, 0.0], now)
        policy.update_connection(True, True, 2.6)
        self.assertEqual(policy.state, SafetyState.ARMED_OUTPUT)
