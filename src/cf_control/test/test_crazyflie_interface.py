import unittest
from unittest.mock import call, Mock

from cf_control.crazyflie_interface import (
    CrazyflieInterface,
    SimulatedCrazyflieInterface,
)
from cf_control.sample_rate import SampleRateLimiter


class CrazyflieInterfaceTest(unittest.TestCase):

    def setUp(self):
        self.scf = Mock()
        self.interface = CrazyflieInterface(self.scf)

    def test_arm_and_disarm_use_supervisor(self):
        self.interface.arm()
        self.interface.disarm()

        self.assertEqual(
            self.scf.cf.supervisor.send_arming_request.call_args_list,
            [call(True), call(False)],
        )

    def test_velocity_uses_world_frame_setpoint(self):
        self.interface.send_velocity(0.1, -0.2, 0.3, 4.0)

        self.scf.cf.commander.send_velocity_world_setpoint.assert_called_once_with(
            0.1,
            -0.2,
            0.3,
            4.0,
        )

    def test_external_position_is_forwarded_to_estimator(self):
        self.interface.send_external_position(0.1, -0.2, 0.3)

        self.scf.cf.extpos.send_extpos.assert_called_once_with(
            0.1,
            -0.2,
            0.3,
        )

    def test_stop_and_disarm_sends_all_safety_commands(self):
        parent = Mock()
        parent.attach_mock(
            self.scf.cf.commander.send_stop_setpoint,
            'stop',
        )
        parent.attach_mock(
            self.scf.cf.commander.send_notify_setpoint_stop,
            'notify',
        )
        parent.attach_mock(
            self.scf.cf.supervisor.send_arming_request,
            'arming',
        )

        self.interface.stop_and_disarm()

        self.assertEqual(
            parent.mock_calls,
            [call.stop(), call.notify(), call.arming(False)],
        )


class SimulatedCrazyflieInterfaceTest(unittest.TestCase):

    def test_commands_only_move_simulation_while_armed(self):
        callback = Mock()
        interface = SimulatedCrazyflieInterface(callback)

        interface.send_velocity(0.1, 0.2, 0.3)
        callback.assert_not_called()

        interface.arm()
        interface.send_velocity(0.1, 0.2, 0.3)
        callback.assert_called_once_with(0.1, 0.2, 0.3, 0.0)

    def test_stop_and_disarm_publishes_zero_velocity(self):
        callback = Mock()
        interface = SimulatedCrazyflieInterface(callback)
        interface.arm()

        interface.stop_and_disarm()

        callback.assert_called_once_with(0.0, 0.0, 0.0, 0.0)
        self.assertFalse(interface.armed)


class SampleRateLimiterTest(unittest.TestCase):

    def test_accepts_only_one_sample_per_period(self):
        limiter = SampleRateLimiter(10.0)

        self.assertTrue(limiter.accept(0.0))
        self.assertFalse(limiter.accept(0.05))
        self.assertTrue(limiter.accept(0.10))

    def test_rejects_invalid_rate(self):
        for rate_hz in (0.0, -1.0, float('inf'), float('nan')):
            with self.subTest(rate_hz=rate_hz):
                with self.assertRaises(ValueError):
                    SampleRateLimiter(rate_hz)


if __name__ == '__main__':
    unittest.main()
