"""Three-drone hover, triangle rotation, line recovery, and landing."""

import math


MISSION_NAME = 'three_drone_demo'

FLIGHT_HEIGHT_M = 1
TRIANGLE_RADIUS_M = 0.4
LINE_SPACING_M = 0.40
ROTATION_DEG = 90.0
ROTATION_DURATION_S = 4.0
ARRIVAL_TOLERANCE_M = 0.18
STATIONARY_TIMEOUT_S = 10.0


MISSION = [
    {
        'name': 'INITIAL_HOVER',
        'transition_s': 4.0,
        'hold_s': 3.0,
        'arrival_tolerance_m': ARRIVAL_TOLERANCE_M,
        'timeout_s': 15.0,
        'offsets': {
            'cf1': (0.0, 0.0, FLIGHT_HEIGHT_M),
            'cf2': (0.0, 0.0, FLIGHT_HEIGHT_M),
            'cf3': (0.0, 0.0, FLIGHT_HEIGHT_M),
        },
    },
    {
        'name': 'TRIANGLE_ENTRY',
        'transition_s': 3.0,
        'hold_s': 1.0,
        'arrival_tolerance_m': ARRIVAL_TOLERANCE_M,
        'timeout_s': STATIONARY_TIMEOUT_S,
        'formation_targets': {
            # Match formation_orbit's starting angles exactly so entering the
            # rotation does not create a target discontinuity. The measured
            # launch order is CF2 (left), CF1 (middle), CF3 (right), so CF2
            # receives the left vertex instead of crossing through CF1.
            'cf1': (
                TRIANGLE_RADIUS_M * 0.5,
                -TRIANGLE_RADIUS_M * math.sqrt(3.0) * 0.5,
                FLIGHT_HEIGHT_M,
            ),
            'cf2': (-TRIANGLE_RADIUS_M, 0.0, FLIGHT_HEIGHT_M),
            'cf3': (
                TRIANGLE_RADIUS_M * 0.5,
                TRIANGLE_RADIUS_M * math.sqrt(3.0) * 0.5,
                FLIGHT_HEIGHT_M,
            ),
        },
    },
    {
        'name': 'TRIANGLE_ROTATE_90',
        'hold_s': 0.0,
        'timeout_s': ROTATION_DURATION_S + 3.0,
        'trajectory': {
            'type': 'formation_orbit',
            'radius_m': TRIANGLE_RADIUS_M,
            'height_m': FLIGHT_HEIGHT_M,
            'duration_s': ROTATION_DURATION_S,
            'angle_deg': ROTATION_DEG,
            'start_angles_deg': {
                'cf1': 300.0,
                'cf2': 180.0,
                'cf3': 60.0,
            },
        },
    },
    {
        'name': 'RETURN_TO_LINE',
        'transition_s': 3.0,
        'hold_s': 1.0,
        'arrival_tolerance_m': ARRIVAL_TOLERANCE_M,
        'timeout_s': STATIONARY_TIMEOUT_S,
        'formation_targets': {
            'cf1': (0.0, 0.0, FLIGHT_HEIGHT_M),
            'cf2': (-LINE_SPACING_M, 0.0, FLIGHT_HEIGHT_M),
            'cf3': (LINE_SPACING_M, 0.0, FLIGHT_HEIGHT_M),
        },
    },
]
