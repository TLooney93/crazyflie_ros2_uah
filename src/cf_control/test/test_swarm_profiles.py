"""Tests for named swarm vehicle rosters."""

from cf_control.swarm_missions import load_swarm_mission
from cf_control.swarm_profiles import load_swarm_profile
import pytest


def test_two_drone_profile_matches_optional_hardware_configuration():
    """The lab profile should load when hardware members are disabled."""
    profile = load_swarm_profile('two_drone_lab')

    assert profile['drone_ids'] == ['cf1', 'cf2']
    assert profile['mission_name'] == 'two_drone_lab'
    assert profile['vicon_topics'] == {
        'cf1': '/vicon/cf1/cf1',
        'cf2': '/vicon/cf2/cf2',
    }
    radio_uris = list(profile['radio_uris'].values())
    if all(radio_uris):
        assert all(uri.startswith('radio://0/') for uri in radio_uris)
        assert len(set(radio_uris)) == 2
        load_swarm_profile('two_drone_lab', require_radio_uris=True)
    else:
        with pytest.raises(ValueError, match='missing radio URIs'):
            load_swarm_profile(
                'two_drone_lab',
                require_radio_uris=True,
            )
    mission = load_swarm_mission(
        profile['mission_name'],
        profile['drone_ids'],
    )
    assert any(phase['name'] == 'ORBIT_ENTRY' for phase in mission)
    assert not any(phase['name'] == 'VERTICAL_STACK' for phase in mission)
    assert any(phase['name'] == 'CONTINUOUS_ORBIT' for phase in mission)


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


def test_three_drone_profile_selects_matching_mission_and_world():
    """The three-drone demo should be launchable directly by profile name."""
    profile = load_swarm_profile('three_drone_demo')

    assert profile['drone_ids'] == ['cf1', 'cf2', 'cf3']
    assert profile['mission_name'] == 'three_drone_demo'
    assert profile['takeoff_height_m'] == 0.30
    assert profile['simulation_world'] == 'crazyflie_three_swarm_world.sdf'
    assert profile['vicon_topics']['cf3'] == '/vicon/cf3/cf3'
    assert load_swarm_mission(
        profile['mission_name'],
        profile['drone_ids'],
    )


def test_three_drone_hardware_configuration_is_valid_when_complete():
    """Configured three-drone radio addresses should be unique."""
    profile = load_swarm_profile('three_drone_demo')

    radio_uris = list(profile['radio_uris'].values())
    if all(radio_uris):
        assert all(uri.startswith('radio://0/') for uri in radio_uris)
        assert len(set(radio_uris)) == 3
        load_swarm_profile('three_drone_demo', require_radio_uris=True)
    else:
        with pytest.raises(ValueError, match='missing radio URIs'):
            load_swarm_profile(
                'three_drone_demo',
                require_radio_uris=True,
            )


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
