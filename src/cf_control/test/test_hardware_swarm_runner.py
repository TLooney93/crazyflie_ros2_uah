"""Tests for coordinator-independent hardware emergency stopping."""

from types import SimpleNamespace

from cf_control.hardware_swarm_runner import HardwareSwarmRunner


class _Controller:
    def __init__(self, drone_id, failure=None):
        self.drone_id = drone_id
        self.failure = failure
        self.reasons = []

    def _stop_and_disarm(self, reason):
        self.reasons.append(reason)
        if self.failure is not None:
            raise self.failure


def test_hardware_stop_reaches_every_controller():
    """The runner service must stop all radios without the coordinator."""
    controllers = [_Controller('cf1'), _Controller('cf2')]
    runner = SimpleNamespace(controllers=controllers)
    response = SimpleNamespace(success=None, message='')

    result = HardwareSwarmRunner.emergency_stop_callback(
        runner, SimpleNamespace(), response,
    )

    assert result.success
    assert all(controller.reasons for controller in controllers)


def test_hardware_stop_reports_partial_failure_after_trying_all():
    """One failed link must not prevent stopping the remaining links."""
    controllers = [
        _Controller('cf1', RuntimeError('link down')),
        _Controller('cf2'),
    ]
    runner = SimpleNamespace(controllers=controllers)
    response = SimpleNamespace(success=None, message='')

    result = HardwareSwarmRunner.emergency_stop_callback(
        runner, SimpleNamespace(), response,
    )

    assert not result.success
    assert 'cf1: link down' in result.message
    assert controllers[1].reasons
