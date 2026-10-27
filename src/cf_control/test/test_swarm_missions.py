"""Tests for editable swarm mission definitions."""

from cf_control.swarm_missions import load_swarm_mission
import pytest


def test_two_drone_demo_loads_for_expected_drones():
    """The bundled mission should load for its configured two drones."""
    phases = load_swarm_mission('two_drone_demo', ['cf1', 'cf2'])

    assert [phase['name'] for phase in phases] == [
        'SPREAD',
        'FORWARD',
        'RETURN_HOME',
    ]


def test_unknown_mission_is_rejected():
    """Unknown mission names should provide a clear validation error."""
    with pytest.raises(ValueError, match='Unknown swarm mission'):
        load_swarm_mission('missing', ['cf1', 'cf2'])


def test_mission_rejects_wrong_drone_set():
    """Mission definitions must specify every requested drone."""
    with pytest.raises(ValueError, match='must exactly match'):
        load_swarm_mission('two_drone_demo', ['cf1'])


def test_mission_rejects_duplicate_drone_ids():
    """A drone cannot occupy two slots in the same mission."""
    with pytest.raises(ValueError, match='unique'):
        load_swarm_mission('two_drone_demo', ['cf1', 'cf1'])
