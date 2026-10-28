"""Default hover, smooth figure-eight, and return mission."""

from cf_control.config import (
    FLIGHT_TEST_FIGURE_EIGHT_DURATION_S,
    FLIGHT_TEST_FIGURE_EIGHT_RADIUS_M,
    FLIGHT_TEST_HEIGHT_M,
    FLIGHT_TEST_HOVER_S,
)


MISSION_NAME = 'figure_eight'

MISSION = [
    {
        'name': 'HOVER',
        'kind': 'waypoint',
        'offset': (0.0, 0.0, FLIGHT_TEST_HEIGHT_M),
        'hold_s': FLIGHT_TEST_HOVER_S,
    },
    {
        'name': 'FIGURE_EIGHT',
        'kind': 'figure_eight',
        'center_offset': (0.0, 0.0, FLIGHT_TEST_HEIGHT_M),
        'radius_m': FLIGHT_TEST_FIGURE_EIGHT_RADIUS_M,
        'duration_s': FLIGHT_TEST_FIGURE_EIGHT_DURATION_S,
    },
    {
        'name': 'RETURN_HOME',
        'kind': 'waypoint',
        'offset': (0.0, 0.0, FLIGHT_TEST_HEIGHT_M),
        'hold_s': 0.0,
    },
]
