from types import SimpleNamespace
from unittest.mock import Mock

from std_srvs.srv import Trigger

from cf_control.crazyflie_node import CrazyflieNode
from cf_control.flight_state import FlightState


def test_takeoff_is_blocked_when_actuation_is_disabled():
    node = SimpleNamespace(
        enable_actuation=False,
        flight_state=FlightState.READY,
        position_ready=True,
        command_interface=Mock(),
    )

    response = CrazyflieNode.takeoff_callback(
        node,
        Trigger.Request(),
        Trigger.Response(),
    )

    assert response.success is False
    node.command_interface.arm.assert_not_called()
    node.command_interface.takeoff.assert_not_called()

def test_takeoff_arms_and_commands_drone_when_enabled():
    node = SimpleNamespace(
        enable_actuation=True,
        flight_state=FlightState.READY,
        position_ready=True,
        command_interface=Mock(),
        takeoff_height=0.4,
        takeoff_duration_s=2.0,
    )

    response = CrazyflieNode.takeoff_callback(
        node,
        Trigger.Request(),
        Trigger.Response(),
    )

    assert response.success is True
    assert node.flight_state is FlightState.FLYING
    node.command_interface.arm.assert_called_once_with()
    node.command_interface.takeoff.assert_called_once_with(0.4, 2.0)

def test_land_commands_drone_and_changes_state():
    timer = Mock()
    node = SimpleNamespace(
        enable_actuation=True,
        flight_state=FlightState.FLYING,
        command_interface=Mock(),
        land_duration_s=2.0,
        landing_timer=None,
        create_timer=Mock(return_value=timer),
        finish_landing_callback=Mock(),
        flight_controller=Mock(),
    )

    response = CrazyflieNode.land_callback(
        node,
        Trigger.Request(),
        Trigger.Response(),
    )

    assert response.success is True
    assert node.flight_state is FlightState.LANDING
    assert node.landing_timer is timer
    node.command_interface.land.assert_called_once_with(
        height_m=0.0,
        duration_s=2.0,
    )
    node.command_interface.release_low_level_control.assert_called_once_with()

def test_emergency_stop_cancels_timer_and_disarms():
    timer = Mock()
    node = SimpleNamespace(
        enable_actuation=True,
        flight_state=FlightState.FLYING,
        command_interface=Mock(),
        flight_controller=Mock(),
        landing_timer=timer,
        get_logger=Mock(return_value=Mock()),
    )

    response = CrazyflieNode.emergency_stop_callback(
        node,
        Trigger.Request(),
        Trigger.Response(),
    )

    assert response.success is True
    assert node.flight_state is FlightState.DISARMED
    assert node.landing_timer is None
    timer.cancel.assert_called_once_with()
    node.flight_controller.clear_target.assert_called_once_with()
    node.command_interface.stop_high_level.assert_called_once_with()
    node.command_interface.disarm.assert_called_once_with()

def test_finish_landing_stops_and_disarms():
    timer = Mock()
    node = SimpleNamespace(
        landing_timer=timer,
        command_interface=Mock(),
        flight_state=FlightState.LANDING,
        get_logger=Mock(return_value=Mock()),
    )

    CrazyflieNode.finish_landing_callback(node)

    assert node.landing_timer is None
    assert node.flight_state is FlightState.DISARMED
    timer.cancel.assert_called_once_with()
    node.command_interface.stop_high_level.assert_called_once_with()
    node.command_interface.disarm.assert_called_once_with()
