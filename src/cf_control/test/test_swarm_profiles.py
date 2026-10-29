"""Tests for named swarm vehicle rosters."""

from cf_control.swarm_missions import load_swarm_mission
from cf_control.swarm_profiles import load_swarm_profile
import pytest


def test_two_drone_profile_is_hardware_complete():
    """The existing lab profile should be ready for hardware launch."""
    profile = load_swarm_profile(
        'two_drone_lab',
        require_radio_uris=True,
    )

    assert profile['drone_ids'] == ['cf1', 'cf2']
    assert profile['mission_name'] == 'two_drone_lab'
    assert profile['vicon_topics'] == {
        'cf1': '/vicon/cf1/cf1',
        'cf2': '/vicon/cf2/cf2',
    }
    assert profile['radio_uris']['cf1'].startswith('radio://0/')
    assert profile['radio_uris']['cf2'].startswith('radio://0/')
    assert profile['radio_uris']['cf1'] != profile['radio_uris']['cf2']
    mission = load_swarm_mission(
        profile['mission_name'],
        profile['drone_ids'],
    )
    assert any(phase['name'] == 'VERTICAL_STACK' for phase in mission)
    assert any(phase['name'].startswith('ORBIT_') for phase in mission)


def test_five_drone_profile_is_available_for_simulation():
    """Simulation does not require physical radio addresses."""
    profile = load_swarm_profile('five_drone_lab')

    assert profile['drone_ids'] == [
        'cf1',
        'cf2',
        'cf3',
        'cf4',
        'cf5',
    ]
    assert profile['mission_name'] == 'five_drone_demo'


def test_five_drone_hardware_requires_radio_configuration():
    """Hardware launch must fail until every real radio URI is supplied."""
    with pytest.raises(ValueError, match='missing radio URIs'):
        load_swarm_profile(
            'five_drone_lab',
            require_radio_uris=True,
        )


def test_unknown_profile_is_rejected():
    """Unknown profile names should list available configurations."""
    with pytest.raises(ValueError, match='Unknown swarm profile'):
        load_swarm_profile('missing')
