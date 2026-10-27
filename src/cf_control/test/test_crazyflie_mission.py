"""Tests for editable single-Crazyflie mission definitions."""

import copy

from cf_control.crazyflie_mission import (
    CRAZYFLIE_MISSIONS,
    load_crazyflie_mission,
)
import pytest


def test_default_figure_eight_mission_loads():
    """The original autonomous sequence should remain the default mission."""
    phases = load_crazyflie_mission('figure_eight')

    assert [phase['name'] for phase in phases] == [
        'HOVER',
        'FIGURE_EIGHT',
        'RETURN_HOME',
    ]


def test_square_demo_finishes_at_home():
    """The editable waypoint example should return above the launch point."""
    phases = load_crazyflie_mission('square_demo')

    assert phases[-1]['offset'][:2] == (0.0, 0.0)


def test_loaded_mission_is_an_independent_copy():
    """Runtime state must not mutate the source mission definition."""
    phases = load_crazyflie_mission('hover_test')
    phases[0]['hold_s'] = 99.0

    assert CRAZYFLIE_MISSIONS['hover_test'][0]['hold_s'] == 5.0


def test_unknown_mission_is_rejected():
    """Unknown mission names should list valid choices."""
    with pytest.raises(ValueError, match='Unknown Crazyflie mission'):
        load_crazyflie_mission('missing')


def test_invalid_waypoint_is_rejected(monkeypatch):
    """Waypoint targets outside the guarded flight volume are invalid."""
    missions = copy.deepcopy(CRAZYFLIE_MISSIONS)
    missions['unsafe'] = [{
        'name': 'TOO_FAR',
        'kind': 'waypoint',
        'offset': (2.0, 0.0, 0.3),
        'hold_s': 0.0,
    }]
    monkeypatch.setattr(
        'cf_control.crazyflie_mission.CRAZYFLIE_MISSIONS',
        missions,
    )

    with pytest.raises(ValueError, match='horizontal target limit'):
        load_crazyflie_mission('unsafe')


def test_mission_must_return_home_before_landing(monkeypatch):
    """Automatic landing must not begin from a displaced final target."""
    missions = copy.deepcopy(CRAZYFLIE_MISSIONS)
    missions['no_return'] = [{
        'name': 'OFFSET_END',
        'kind': 'waypoint',
        'offset': (0.2, 0.0, 0.3),
        'hold_s': 0.0,
    }]
    monkeypatch.setattr(
        'cf_control.crazyflie_mission.CRAZYFLIE_MISSIONS',
        missions,
    )

    with pytest.raises(ValueError, match='must finish above'):
        load_crazyflie_mission('no_return')
