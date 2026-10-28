"""Small square waypoint demonstration with a return to launch."""

from cf_control.config import FLIGHT_TEST_HEIGHT_M


MISSION_NAME = 'square_demo'

MISSION = [
    {
        'name': 'INITIAL_HOVER',
        'kind': 'waypoint',
        'offset': (0.0, 0.0, FLIGHT_TEST_HEIGHT_M),
        'hold_s': 2.0,
    },
    {
        'name': 'SQUARE_1',
        'kind': 'waypoint',
        'offset': (0.20, 0.0, FLIGHT_TEST_HEIGHT_M),
        'hold_s': 1.0,
    },
    {
        'name': 'SQUARE_2',
        'kind': 'waypoint',
        'offset': (0.20, 0.20, FLIGHT_TEST_HEIGHT_M),
        'hold_s': 1.0,
    },
    {
        'name': 'SQUARE_3',
        'kind': 'waypoint',
        'offset': (0.0, 0.20, FLIGHT_TEST_HEIGHT_M),
        'hold_s': 1.0,
    },
    {
        'name': 'RETURN_HOME',
        'kind': 'waypoint',
        'offset': (0.0, 0.0, FLIGHT_TEST_HEIGHT_M),
        'hold_s': 1.0,
    },
]
