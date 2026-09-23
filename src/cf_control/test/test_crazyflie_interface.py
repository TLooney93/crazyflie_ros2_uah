"""Tests for the Crazyflie command interface."""

from unittest.mock import Mock

from cf_control.crazyflie_interface import CrazyflieInterface


def test_arm_and_disarm_request():
    scf = Mock()
    interface = CrazyflieInterface(scf)

    interface.arm()
    interface.disarm()

    calls = scf.cf.supervisor.send_arming_request.call_args_list
    assert calls[0].args == (True,)
    assert calls[1].args == (False,)

def test_takeoff_command():
    scf = Mock()
    interface = CrazyflieInterface(scf)

    interface.takeoff(0.5, 2.0)

    scf.cf.high_level_commander.takeoff.assert_called_once_with(
        0.5,
        2.0,
        yaw=None,
    )

def test_land_command():
    scf = Mock()
    interface = CrazyflieInterface(scf)

    interface.land(0.0, 2.0)

    scf.cf.high_level_commander.land.assert_called_once_with(
        0.0,
        2.0,
        yaw=None,
    )