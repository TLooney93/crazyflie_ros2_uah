"""Two-drone synchronized orbit laboratory mission."""

MISSION_NAME = 'two_drone_lab'

ORBIT_RADIUS_M = 0.30
ORBIT_HEIGHT_M = 0.45
ORBIT_DURATION_S = 30.0
# Hardware Vicon traces show roughly 5-14 cm of steady tracking error during
# shared-radio flight.  These tolerances are deliberately wider than the
# simulation values, while remaining well inside the separation guard.
STATIONARY_ARRIVAL_TOLERANCE_M = 0.15
STATIONARY_PHASE_TIMEOUT_S = 20.0


MISSION = [
    {
        'name': 'INITIAL_HOVER',
        'hold_s': 2.0,
        'arrival_tolerance_m': STATIONARY_ARRIVAL_TOLERANCE_M,
        'timeout_s': STATIONARY_PHASE_TIMEOUT_S,
        'offsets': {
            'cf1': (0.0, 0.0, ORBIT_HEIGHT_M),
            'cf2': (0.0, 0.0, ORBIT_HEIGHT_M),
        },
    },
    {
        'name': 'ORBIT_ENTRY',
        'hold_s': 2.0,
        'arrival_tolerance_m': STATIONARY_ARRIVAL_TOLERANCE_M,
        'timeout_s': STATIONARY_PHASE_TIMEOUT_S,
        'formation_targets': {
            # Move onto the shared circle before advancing through it.
            'cf1': (-ORBIT_RADIUS_M, 0.0, ORBIT_HEIGHT_M),
            'cf2': (ORBIT_RADIUS_M, 0.0, ORBIT_HEIGHT_M),
        },
    },
    {
        'name': 'CONTINUOUS_ORBIT',
        'hold_s': 0.0,
        'timeout_s': ORBIT_DURATION_S + 5.0,
        'trajectory': {
            'type': 'formation_orbit',
            'radius_m': ORBIT_RADIUS_M,
            'height_m': ORBIT_HEIGHT_M,
            'duration_s': ORBIT_DURATION_S,
        },
    },
    {
        'name': 'FINAL_HOVER',
        'hold_s': 3.0,
        'arrival_tolerance_m': STATIONARY_ARRIVAL_TOLERANCE_M,
        'timeout_s': STATIONARY_PHASE_TIMEOUT_S,
        'offsets': {
            'cf1': (0.0, 0.0, ORBIT_HEIGHT_M),
            'cf2': (0.0, 0.0, ORBIT_HEIGHT_M),
        },
    },
]
