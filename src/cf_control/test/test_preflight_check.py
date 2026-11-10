"""Tests for the read-only Crazyflie hardware preflight command."""

from cf_control.config import DRONES, PREFLIGHT_MIN_BATTERY_VOLTAGE
from cf_control.preflight_check import _assessment, _selected_drones

import pytest


def test_default_selection_checks_every_configured_drone():
    """No arguments should cover the complete configured fleet."""
    assert _selected_drones([]) == list(DRONES)


def test_explicit_selection_is_preserved():
    """Named subsets should retain the operator's requested order."""
    assert _selected_drones(['cf3', 'cf1']) == ['cf3', 'cf1']


def test_unknown_drone_is_rejected_before_radio_access():
    """Invalid names must fail before initializing radio hardware."""
    with pytest.raises(ValueError, match='Unknown drone'):
        _selected_drones(['missing'])


def test_assessment_rejects_low_battery_and_supervisor_faults():
    """Low voltage, fault states, and armed state must fail preflight."""
    low = PREFLIGHT_MIN_BATTERY_VOLTAGE - 0.01

    assert _assessment(4.1, {'Can fly'}) == 'PASS'
    assert 'battery below' in _assessment(low, set())
    assert 'Is tumbled' in _assessment(4.1, {'Is tumbled'})
    assert 'unexpectedly armed' in _assessment(4.1, {'Is armed'})
