"""Two-drone stacking and synchronized orbit laboratory mission."""

import math


MISSION_NAME = 'two_drone_lab'

ORBIT_RADIUS_M = 0.30
ORBIT_HEIGHT_M = 0.40
ORBIT_STEPS = 12


def _orbit_phase(step):
    """Create one synchronized orbit waypoint."""
    angle = 2.0 * math.pi * step / ORBIT_STEPS
    cos_angle = math.cos(angle)
    sin_angle = math.sin(angle)

    return {
        'name': f'ORBIT_{step:02d}',
        'hold_s': 0.0,
        'offsets': {
            # Homes are opposite sides of a 0.30 m shared orbit center.
            'cf1': (
                ORBIT_RADIUS_M * (1.0 - cos_angle),
                -ORBIT_RADIUS_M * sin_angle,
                ORBIT_HEIGHT_M,
            ),
            'cf2': (
                ORBIT_RADIUS_M * (cos_angle - 1.0),
                ORBIT_RADIUS_M * sin_angle,
                ORBIT_HEIGHT_M,
            ),
        },
    }


MISSION = [
    {
        'name': 'INITIAL_HOVER',
        'hold_s': 2.0,
        'offsets': {
            'cf1': (0.0, 0.0, 0.40),
            'cf2': (0.0, 0.0, 0.40),
        },
    },
    {
        'name': 'VERTICAL_STACK',
        'hold_s': 3.0,
        'offsets': {
            # With homes at x=-0.30/+0.30, both move above x=0.
            'cf1': (0.30, 0.0, 0.30),
            'cf2': (-0.30, 0.0, 0.65),
        },
    },
    {
        'name': 'UNSTACK',
        'hold_s': 2.0,
        'offsets': {
            'cf1': (0.0, 0.0, 0.40),
            'cf2': (0.0, 0.0, 0.40),
        },
    },
    *[_orbit_phase(step) for step in range(ORBIT_STEPS + 1)],
    {
        'name': 'FINAL_HOVER',
        'hold_s': 3.0,
        'offsets': {
            'cf1': (0.0, 0.0, 0.40),
            'cf2': (0.0, 0.0, 0.40),
        },
    },
]
