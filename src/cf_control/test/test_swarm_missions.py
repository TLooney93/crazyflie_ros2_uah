"""Tests for editable swarm mission definitions."""

import math
from types import SimpleNamespace

from cf_control.config import SWARM_MIN_SEPARATION_M
from cf_control.swarm_missions import load_swarm_mission
from cf_control.swarm_node import SwarmNode

import pytest


class _RecordingLogger:
    """Collect coordinator messages in unbound callback tests."""

    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(message)

    def warning(self, message):
        self.messages.append(message)


def test_two_drone_lab_loads_orbit_for_expected_drones():
    """The bundled mission should load for its configured two drones."""
    phases = load_swarm_mission('two_drone_lab', ['cf1', 'cf2'])

    assert phases[0]['name'] == 'INITIAL_HOVER'
    assert phases[-1]['name'] == 'FINAL_HOVER'
    assert all(
        'trajectory' in phase
        or set(phase.get('offsets', phase.get('formation_targets', {})))
        == {'cf1', 'cf2'}
        for phase in phases
    )
    assert 'ORBIT_ENTRY' in [phase['name'] for phase in phases]
    assert 'CONTINUOUS_ORBIT' in [phase['name'] for phase in phases]
    assert 'VERTICAL_STACK' not in [phase['name'] for phase in phases]
    waypoint_orbits = [
        phase
        for phase in phases
        if phase['name'].removeprefix('ORBIT_').isdigit()
    ]
    assert waypoint_orbits == []
    orbit = next(
        phase for phase in phases if phase['name'] == 'CONTINUOUS_ORBIT'
    )
    assert orbit['trajectory'] == {
        'type': 'formation_orbit',
        'radius_m': 0.30,
        'height_m': 0.45,
        'duration_s': 30.0,
    }
    stationary = [phase for phase in phases if 'trajectory' not in phase]
    assert all(phase['arrival_tolerance_m'] == 0.15 for phase in stationary)
    assert all(phase['timeout_s'] == 20.0 for phase in stationary)


def test_two_drone_lab_orbit_entry_keeps_horizontal_separation():
    """Orbit entry must keep both drones level and safely separated."""
    phases = load_swarm_mission('two_drone_lab', ['cf1', 'cf2'])
    entry = next(phase for phase in phases if phase['name'] == 'ORBIT_ENTRY')

    cf1 = entry['formation_targets']['cf1']
    cf2 = entry['formation_targets']['cf2']

    lateral_separation = math.hypot(
        cf2[0] - cf1[0],
        cf2[1] - cf1[1],
    )
    assert lateral_separation == pytest.approx(0.60)
    assert cf2[2] == cf1[2]
    assert math.dist(cf1, cf2) > SWARM_MIN_SEPARATION_M


def test_formation_targets_follow_reversed_measured_launch_baseline():
    """Formation geometry must not assume cf1 starts on the left."""
    node = object.__new__(SwarmNode)
    node.drone_ids = ['cf1', 'cf2']
    node.home_positions = {
        'cf1': (0.34, 0.08, 0.05),
        'cf2': (-0.48, 0.17, 0.06),
    }
    phases = load_swarm_mission('two_drone_lab', node.drone_ids)
    entry = next(p for p in phases if p['name'] == 'ORBIT_ENTRY')

    targets = node._targets_for_phase(entry)

    assert math.dist(targets['cf1'][:2], targets['cf2'][:2]) == pytest.approx(
        0.60
    )
    assert targets['cf2'][2] - targets['cf1'][2] == pytest.approx(0.01)


def test_orbit_targets_share_measured_formation_midpoint():
    """Both orbit targets must remain opposite around one measured center."""
    node = object.__new__(SwarmNode)
    node.drone_ids = ['cf1', 'cf2']
    node.home_positions = {
        'cf1': (0.34, 0.08, 0.05),
        'cf2': (-0.48, 0.17, 0.06),
    }
    phases = load_swarm_mission('two_drone_lab', node.drone_ids)
    orbit = next(p for p in phases if p['name'] == 'CONTINUOUS_ORBIT')

    targets, feedforwards = node._trajectory_commands(orbit, 7.5)
    target_midpoint = tuple(
        (first + second) / 2.0
        for first, second in zip(targets['cf1'][:2], targets['cf2'][:2])
    )
    home_midpoint = tuple(
        (first + second) / 2.0
        for first, second in zip(
            node.home_positions['cf1'][:2],
            node.home_positions['cf2'][:2],
        )
    )

    assert target_midpoint == pytest.approx(home_midpoint)
    assert math.dist(targets['cf1'][:2], targets['cf2'][:2]) == pytest.approx(
        0.60
    )
    expected_speed = 2.0 * math.pi * 0.30 / 30.0
    assert math.hypot(*feedforwards['cf1'][:2]) == pytest.approx(
        expected_speed
    )
    assert math.hypot(*feedforwards['cf2'][:2]) == pytest.approx(
        expected_speed
    )
    assert feedforwards['cf1'] == pytest.approx(
        tuple(-value for value in feedforwards['cf2'])
    )


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


def test_three_drone_demo_has_bounded_triangle_sequence():
    """The demo must hover, rotate a triangle 90 degrees, and form a line."""
    drone_ids = ['cf1', 'cf2', 'cf3']
    phases = load_swarm_mission('three_drone_demo', drone_ids)

    assert [phase['name'] for phase in phases] == [
        'INITIAL_HOVER',
        'TRIANGLE_ENTRY',
        'TRIANGLE_ROTATE_90',
        'RETURN_TO_LINE',
    ]
    assert phases[0]['hold_s'] == pytest.approx(3.0)
    assert all(set(phase['active_drones']) == set(drone_ids) for phase in phases)
    rotation = phases[2]['trajectory']
    assert rotation['angle_deg'] == pytest.approx(90.0)
    assert rotation['duration_s'] == pytest.approx(4.0)
    assert sum(phase['hold_s'] for phase in phases) + rotation['duration_s'] < 10.0


def test_three_drone_rotation_ends_at_ninety_degrees():
    """The continuous trajectory must stop after one quarter turn."""
    node = object.__new__(SwarmNode)
    node.drone_ids = ['cf1', 'cf2', 'cf3']
    node.home_positions = {
        'cf1': (-0.8, 0.0, 0.04),
        'cf2': (0.0, 0.0, 0.04),
        'cf3': (0.8, 0.0, 0.04),
    }
    phase = load_swarm_mission(
        'three_drone_demo', node.drone_ids,
    )[2]

    start, _ = node._trajectory_commands(phase, 0.0)
    end, _ = node._trajectory_commands(
        phase, phase['trajectory']['duration_s'],
    )

    assert start['cf1'][:2] == pytest.approx((-0.3, 0.0))
    assert end['cf1'][:2] == pytest.approx((0.0, -0.3))
    assert math.dist(end['cf1'], end['cf2']) > SWARM_MIN_SEPARATION_M


def test_staged_phase_arrival_ignores_grounded_members():
    """Only active drones participate in staged phase arrival timing."""
    node = object.__new__(SwarmNode)
    node.drone_ids = ['cf1', 'cf2', 'cf3']
    node.mission = load_swarm_mission(
        'three_drone_demo', node.drone_ids,
    )
    node.phase_index = 0
    node.phase_started_at = 10.0
    node.phase_arrived_at = None
    node.phase_arrived_drones = set()
    node.staged_mission = True
    node.mission_active_drones = {'cf1'}
    node.home_positions = {
        'cf1': (0.0, 0.0, 0.0),
        'cf2': (1.0, 0.0, 0.0),
        'cf3': (2.0, 0.0, 0.0),
    }
    node.positions = {
        'cf1': (0.0, 0.0, 0.30),
        'cf2': (1.0, 0.0, 0.0),
        'cf3': (2.0, 0.0, 0.0),
    }
    node._publish_motion_commands = lambda targets, feedforwards: None
    node._advance_mission_phase = lambda now: None

    node._update_mission(10.1)

    assert node.phase_arrived_drones == {'cf1'}
    assert node.phase_arrived_at == pytest.approx(10.1)


def test_staged_mission_rejects_rising_inactive_member():
    """A grounded marker rising exposes a radio/Vicon identity mismatch."""
    node = object.__new__(SwarmNode)
    node.drone_ids = ['cf1', 'cf2', 'cf3']
    node.mission_active_drones = {'cf1'}
    node.phase_transition_drones = set()
    node.home_positions = {
        'cf1': (0.0, 0.0, 0.02),
        'cf2': (1.0, 0.0, 0.02),
        'cf3': (2.0, 0.0, 0.02),
    }
    node.positions = {
        'cf1': (0.0, 0.0, 0.25),
        'cf2': (1.0, 0.0, 0.02),
        'cf3': (2.0, 0.0, 0.15),
    }

    error = node._uncommanded_motion_error()

    assert 'inactive cf3 rose' in error
    assert 'radio/Vicon identity mismatch' in error


def test_staged_mission_allows_commanded_member_to_rise():
    """The identity guard must not reject the drone being launched."""
    node = object.__new__(SwarmNode)
    node.drone_ids = ['cf1', 'cf2', 'cf3']
    node.mission_active_drones = set()
    node.phase_transition_drones = {'cf1'}
    node.home_positions = {
        drone_id: (float(index), 0.0, 0.02)
        for index, drone_id in enumerate(node.drone_ids)
    }
    node.positions = dict(node.home_positions)
    node.positions['cf1'] = (0.0, 0.0, 0.25)

    assert node._uncommanded_motion_error() is None


def test_staged_transition_keeps_retained_airborne_member_alive():
    """Landing one member must not starve another member's heartbeat."""
    node = object.__new__(SwarmNode)
    node.drone_ids = ['cf1', 'cf2', 'cf3']
    node.mission = [{
        'name': 'HOVER CF1/CF3',
        'offsets': {
            'cf1': (0.0, 0.0, 0.30),
            'cf2': (0.0, 0.0, 0.0),
            'cf3': (0.0, 0.0, 0.30),
        },
    }]
    node.phase_index = 0
    node.swarm_state = 'PHASE_LANDING'
    node.mission_active_drones = {'cf1', 'cf2'}
    node.phase_transition_drones = {'cf2'}
    node.home_positions = {
        'cf1': (0.0, 0.0, 0.04),
        'cf2': (-1.0, 0.0, 0.04),
        'cf3': (1.0, 0.0, 0.04),
    }
    published = []
    node._publish_motion_commands = (
        lambda targets, feedforwards: published.append(
            (targets, feedforwards)
        )
    )

    node._publish_staged_transition_heartbeat()

    targets, feedforwards = published[0]
    assert set(targets) == {'cf1'}
    assert targets['cf1'] == pytest.approx((0.0, 0.0, 0.34))
    assert feedforwards == {'cf1': (0.0, 0.0, 0.0)}


def test_mission_rejects_wrong_drone_set():
    """Mission definitions must specify every requested drone."""
    with pytest.raises(ValueError, match='must exactly match'):
        load_swarm_mission('two_drone_lab', ['cf1'])


def test_mission_rejects_duplicate_drone_ids():
    """A drone cannot occupy two slots in the same mission."""
    with pytest.raises(ValueError, match='unique'):
        load_swarm_mission('two_drone_lab', ['cf1', 'cf1'])


def test_takeoff_readiness_requires_every_controller_hover_phase():
    """Position tolerance alone must not advance the swarm mission."""
    node = object.__new__(SwarmNode)
    node.drone_ids = ['cf1', 'cf2', 'cf3']
    node.control_phases = {
        'cf1': 'EXTERNAL_HOVER',
        'cf2': 'EXTERNAL_TAKEOFF',
        'cf3': 'EXTERNAL_HOVER',
    }

    assert not node._all_control_phases_are('EXTERNAL_HOVER')

    node.control_phases['cf2'] = 'EXTERNAL_HOVER'
    assert node._all_control_phases_are('EXTERNAL_HOVER')


def test_side_to_side_starts_mission_after_full_wand_hover():
    """The physical gesture should hand a stable swarm to its mission."""
    logger = _RecordingLogger()
    mission_started = []
    node = SimpleNamespace(
        pending_action=None,
        swarm_state='WAND_ACTIVE',
        drone_ids=['cf1', 'cf2', 'cf3'],
        drone_states={
            'cf1': 'FLYING',
            'cf2': 'FLYING',
            'cf3': 'FLYING',
        },
        mission_name='three_drone_demo',
        run_full_sequence=False,
        get_logger=lambda: logger,
        _positions_are_fresh=lambda: True,
        _all_control_phases_are=lambda phase: phase == 'WAND_HOVER',
        _mission_targets_are_separated=lambda: True,
        _begin_mission=lambda: mission_started.append(True),
        _safety_land=lambda reason: None,
    )

    SwarmNode._on_wand_gesture(
        node,
        SimpleNamespace(data='side_to_side'),
    )

    assert node.run_full_sequence
    assert mission_started == [True]


def test_side_to_side_is_ignored_before_every_drone_reaches_hover():
    """A partial swarm takeoff must not transition into choreography."""
    logger = _RecordingLogger()
    mission_started = []
    node = SimpleNamespace(
        pending_action=None,
        swarm_state='WAND_ACTIVE',
        drone_ids=['cf1', 'cf2', 'cf3'],
        drone_states={
            'cf1': 'FLYING',
            'cf2': 'FLYING',
            'cf3': 'FLYING',
        },
        mission_name='three_drone_demo',
        run_full_sequence=False,
        get_logger=lambda: logger,
        _positions_are_fresh=lambda: True,
        _all_control_phases_are=lambda phase: False,
        _mission_targets_are_separated=lambda: True,
        _begin_mission=lambda: mission_started.append(True),
        _safety_land=lambda reason: None,
    )

    SwarmNode._on_wand_gesture(
        node,
        SimpleNamespace(data='side_to_side'),
    )

    assert not node.run_full_sequence
    assert mission_started == []


def test_sustained_altitude_collapse_is_reported():
    """A member that falls after takeoff must abort the active mission."""
    node = object.__new__(SwarmNode)
    node.drone_ids = ['cf1', 'cf2']
    node.takeoff_height_m = 0.45
    node.home_positions = {
        'cf1': (0.0, 0.0, 0.05),
        'cf2': (0.6, 0.0, 0.05),
    }
    node.positions = {
        'cf1': (0.0, 0.0, 0.08),
        'cf2': (0.6, 0.0, 0.48),
    }
    node.altitude_violation_started_at = {}

    assert node._altitude_safety_error(10.0) is None
    error = node._altitude_safety_error(10.5)

    assert 'Sustained altitude loss: cf1' in error
