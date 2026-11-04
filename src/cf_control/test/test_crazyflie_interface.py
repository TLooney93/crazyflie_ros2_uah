from threading import Event
import time
import unittest
from unittest.mock import call, Mock

from cf_control.crazyflie_interface import (
    CrazyflieInterface,
    RadioCommandDispatcher,
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

    def test_async_worker_keeps_callback_nonblocking_and_uses_latest_pose(
        self,
    ):
        """A full cflib queue must not stall ROS or replay old Vicon poses."""
        velocity_started = Event()
        release_velocity = Event()
        position_sent = Event()

        def block_velocity(*_args):
            velocity_started.set()
            release_velocity.wait(timeout=1.0)

        def record_position(*_args):
            position_sent.set()

        self.scf.cf.commander.send_velocity_world_setpoint.side_effect = (
            block_velocity
        )
        self.scf.cf.extpos.send_extpos.side_effect = record_position
        self.interface.start()
        try:
            started_at = time.monotonic()
            self.interface.send_velocity(0.1, 0.0, 0.2)
            elapsed = time.monotonic() - started_at

            self.assertLess(elapsed, 0.05)
            self.assertTrue(velocity_started.wait(timeout=0.5))

            self.interface.send_external_position(1.0, 1.0, 1.0)
            self.interface.send_external_position(2.0, 2.0, 2.0)
            self.interface.send_external_position(3.0, 3.0, 3.0)
            release_velocity.set()

            self.assertTrue(position_sent.wait(timeout=0.5))
            self.scf.cf.extpos.send_extpos.assert_called_once_with(
                3.0,
                3.0,
                3.0,
            )
        finally:
            release_velocity.set()
            self.interface.shutdown()

    def test_safety_command_invalidates_worker_update_already_dequeued(self):
        """A stale worker command must not be transmitted after a stop."""
        generation = self.interface._command_generation

        self.interface._clear_pending()
        self.interface._send_velocity_if_current(
            generation,
            0.1,
            0.0,
            0.2,
            0.0,
        )

        self.scf.cf.commander.send_velocity_world_setpoint.assert_not_called()

    def test_shared_dispatcher_prioritizes_all_velocities_before_poses(self):
        """One blocked link must not let poses jump ahead of flight refreshes."""
        dispatcher = RadioCommandDispatcher()
        first_scf = Mock()
        second_scf = Mock()
        first = CrazyflieInterface(first_scf, dispatcher=dispatcher)
        second = CrazyflieInterface(second_scf, dispatcher=dispatcher)
        velocity_started = Event()
        release_velocity = Event()
        complete = Event()
        order = []

        def first_velocity(*_args):
            order.append('velocity_cf1')
            velocity_started.set()
            release_velocity.wait(timeout=1.0)

        def second_velocity(*_args):
            order.append('velocity_cf2')

        def first_pose(*_args):
            order.append('pose_cf1')

        def second_pose(*_args):
            order.append('pose_cf2')
            complete.set()

        first_scf.cf.commander.send_velocity_world_setpoint.side_effect = (
            first_velocity
        )
        second_scf.cf.commander.send_velocity_world_setpoint.side_effect = (
            second_velocity
        )
        first_scf.cf.extpos.send_extpos.side_effect = first_pose
        second_scf.cf.extpos.send_extpos.side_effect = second_pose
        first.start()
        second.start()
        try:
            first.send_velocity(0.1, 0.0, 0.2)
            self.assertTrue(velocity_started.wait(timeout=0.5))
            first.send_external_position(1.0, 1.0, 1.0)
            second.send_external_position(2.0, 2.0, 2.0)
            second.send_velocity(0.2, 0.0, 0.2)
            release_velocity.set()

            self.assertTrue(complete.wait(timeout=0.5))
            self.assertEqual(
                order,
                ['velocity_cf1', 'velocity_cf2', 'pose_cf1', 'pose_cf2'],
            )
        finally:
            release_velocity.set()
            first.shutdown()
            second.shutdown()
            dispatcher.shutdown()


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
