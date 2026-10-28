"""Vertical takeoff, stationary hover, and landing test mission."""

from cf_control.config import FLIGHT_TEST_HEIGHT_M


MISSION_NAME = 'hover_test'

MISSION = [
    {
        'name': 'HOVER',
        'kind': 'waypoint',
        'offset': (0.0, 0.0, FLIGHT_TEST_HEIGHT_M),
        'hold_s': 5.0,
    },
]
