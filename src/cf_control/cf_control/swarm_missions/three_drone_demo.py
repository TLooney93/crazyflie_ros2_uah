"""Three-drone hover, triangle formation, and orbit demonstration."""

import math


MISSION_NAME = 'three_drone_demo'

FLIGHT_HEIGHT_M = 0.45
TRIANGLE_RADIUS_M = 0.35
ORBIT_DURATION_S = 30.0
ARRIVAL_TOLERANCE_M = 0.15
PHASE_TIMEOUT_S = 20.0


def _stationary_phase(name, targets, hold_s=2.0):
    return {
        'name': name,
        'hold_s': hold_s,
        'arrival_tolerance_m': ARRIVAL_TOLERANCE_M,
        'timeout_s': PHASE_TIMEOUT_S,
        'formation_targets': targets,
    }


# At orbit angle zero these vertices exactly match the generic equally spaced
# formation-orbit trajectory, preventing a setpoint jump at orbit entry.
TRIANGLE_TARGETS = {
    'cf1': (-TRIANGLE_RADIUS_M, 0.0, FLIGHT_HEIGHT_M),
    'cf2': (
        TRIANGLE_RADIUS_M / 2.0,
        -math.sqrt(3.0) * TRIANGLE_RADIUS_M / 2.0,
        FLIGHT_HEIGHT_M,
    ),
    'cf3': (
        TRIANGLE_RADIUS_M / 2.0,
        math.sqrt(3.0) * TRIANGLE_RADIUS_M / 2.0,
        FLIGHT_HEIGHT_M,
    ),
}


MISSION = [
    {
        'name': 'INITIAL_HOVER',
        'hold_s': 2.0,
        'arrival_tolerance_m': ARRIVAL_TOLERANCE_M,
        'timeout_s': PHASE_TIMEOUT_S,
        'offsets': {
            'cf1': (0.0, 0.0, FLIGHT_HEIGHT_M),
            'cf2': (0.0, 0.0, FLIGHT_HEIGHT_M),
            'cf3': (0.0, 0.0, FLIGHT_HEIGHT_M),
        },
    },
    _stationary_phase('TRIANGLE_ENTRY', TRIANGLE_TARGETS, hold_s=3.0),
    {
        'name': 'TRIANGLE_ORBIT',
        'hold_s': 0.0,
        'timeout_s': ORBIT_DURATION_S + 5.0,
        'trajectory': {
            'type': 'formation_orbit',
            'radius_m': TRIANGLE_RADIUS_M,
            'height_m': FLIGHT_HEIGHT_M,
            'duration_s': ORBIT_DURATION_S,
        },
    },
]
