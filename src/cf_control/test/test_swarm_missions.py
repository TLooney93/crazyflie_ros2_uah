"""Tests for editable swarm mission definitions."""

import math

from cf_control.config import SWARM_MIN_SEPARATION_M
from cf_control.swarm_missions import load_swarm_mission
import pytest


def test_two_drone_lab_loads_orbit_stack_for_expected_drones():
    """The bundled mission should load for its configured two drones."""
    phases = load_swarm_mission('two_drone_lab', ['cf1', 'cf2'])

    assert phases[0]['name'] == 'INITIAL_HOVER'
    assert phases[-1]['name'] == 'FINAL_HOVER'
    assert all(set(phase['offsets']) == {'cf1', 'cf2'} for phase in phases)
    assert 'VERTICAL_STACK' in [phase['name'] for phase in phases]
    orbit = [p for p in phases if p['name'].startswith('ORBIT_')]
    assert len(orbit) == 16
    assert orbit[0]['name'] == 'ORBIT_01'
    assert orbit[-1]['name'] == 'ORBIT_16'
    assert all(phase['arrival_tolerance_m'] == 0.10 for phase in orbit)
    assert all(phase['timeout_s'] == 8.0 for phase in orbit)
    stationary = [phase for phase in phases if not phase['name'].startswith('ORBIT_')]
    assert all(phase['arrival_tolerance_m'] == 0.15 for phase in stationary)
    assert all(phase['timeout_s'] == 20.0 for phase in stationary)


def test_two_drone_lab_stack_is_staggered_away_from_rotor_wash():
    """The upper stack target must not sit directly over the lower drone."""
    phases = load_swarm_mission('two_drone_lab', ['cf1', 'cf2'])
    stack = next(phase for phase in phases if phase['name'] == 'VERTICAL_STACK')

    # Hardware homes are x=-0.30/+0.30 m in the lab setup.
    cf1 = tuple(
        home + offset
        for home, offset in zip((-0.30, 0.0, 0.0), stack['offsets']['cf1'])
    )
    cf2 = tuple(
        home + offset
        for home, offset in zip((0.30, 0.0, 0.0), stack['offsets']['cf2'])
    )

    lateral_separation = math.hypot(
        cf2[0] - cf1[0],
        cf2[1] - cf1[1],
    )
    assert lateral_separation == pytest.approx(0.45)
    assert cf2[2] - cf1[2] >= 0.70
    assert math.dist(cf1, cf2) > SWARM_MIN_SEPARATION_M


def test_two_drone_demo_remains_a_compatibility_alias():
    """Existing mission overrides should continue to load."""
    lab = load_swarm_mission('two_drone_lab', ['cf1', 'cf2'])
    demo = load_swarm_mission('two_drone_demo', ['cf1', 'cf2'])

    assert demo == lab


def test_unknown_mission_is_rejected():
    """Unknown mission names should provide a clear validation error."""
    with pytest.raises(ValueError, match='Unknown swarm mission'):
        load_swarm_mission('missing', ['cf1', 'cf2'])


def test_five_drone_demo_loads_for_expected_drones():
    """The five-drone profile should have a matching saved mission."""
    drone_ids = ['cf1', 'cf2', 'cf3', 'cf4', 'cf5']
    phases = load_swarm_mission('five_drone_demo', drone_ids)

    assert [phase['name'] for phase in phases] == [
        'HOVER',
        'FORWARD',
        'RETURN_HOME',
    ]


def test_mission_rejects_wrong_drone_set():
    """Mission definitions must specify every requested drone."""
    with pytest.raises(ValueError, match='must exactly match'):
        load_swarm_mission('two_drone_lab', ['cf1'])


def test_mission_rejects_duplicate_drone_ids():
    """A drone cannot occupy two slots in the same mission."""
    with pytest.raises(ValueError, match='unique'):
        load_swarm_mission('two_drone_lab', ['cf1', 'cf1'])
