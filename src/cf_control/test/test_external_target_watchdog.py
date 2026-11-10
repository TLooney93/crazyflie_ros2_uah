"""Tests for the coordinator target-heartbeat watchdog."""

from types import SimpleNamespace

from cf_control.config import EXTERNAL_TARGET_TIMEOUT_S
from cf_control.crazyflie_node import CrazyflieNode


def test_watchdog_waits_for_first_coordinator_target():
    """An early-hovering drone must wait while peers finish taking off."""
    node = SimpleNamespace(external_target_received_at=None)

    assert not CrazyflieNode._external_target_heartbeat_timed_out(node, 50.0)


def test_watchdog_expires_after_active_target_stream_stops():
    """Once started, a stale coordinator target must still cause landing."""
    node = SimpleNamespace(external_target_received_at=10.0)

    assert not CrazyflieNode._external_target_heartbeat_timed_out(
        node, 10.0 + EXTERNAL_TARGET_TIMEOUT_S,
    )
    assert CrazyflieNode._external_target_heartbeat_timed_out(
        node, 10.0 + EXTERNAL_TARGET_TIMEOUT_S + 0.01,
    )
