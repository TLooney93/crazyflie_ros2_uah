import unittest
from unittest.mock import Mock, call

from cf_control.crazyflie_interface import (
    CrazyflieInterface,
    SimulatedCrazyflieInterface,
)


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


if __name__ == '__main__':
    unittest.main()
