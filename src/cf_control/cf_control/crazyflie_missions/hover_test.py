"""Vertical takeoff, stationary hover, and landing test mission."""

MISSION_NAME = 'hover_test'

MISSION = [
    {
        'name': 'HOVER',
        'kind': 'waypoint',
        'offset': (0.0, 0.0, 0.3),
        'hold_s': 5.0,
    },
]
