"""Two-drone stacking and synchronized orbit laboratory mission."""

import math


MISSION_NAME = 'two_drone_lab'

ORBIT_RADIUS_M = 0.30
ORBIT_HEIGHT_M = 0.45
ORBIT_STEPS = 16
# Hardware Vicon traces show roughly 5-14 cm of steady tracking error during
# shared-radio flight.  These tolerances are deliberately wider than the
# simulation values, while remaining well inside the separation guard.
STATIONARY_ARRIVAL_TOLERANCE_M = 0.15
ORBIT_ARRIVAL_TOLERANCE_M = 0.10
STATIONARY_PHASE_TIMEOUT_S = 20.0
ORBIT_PHASE_TIMEOUT_S = 8.0

# Keep the aircraft vertically separated without placing the lower vehicle
# directly in the upper vehicle's rotor wash.  The resulting target-to-target
# separation is about 0.83 m (0.45 m lateral and 0.70 m vertical), well inside
# a 10 ft / 3.05 m test volume.
STACK_LATERAL_SEPARATION_M = 0.45
STACK_INWARD_OFFSET_M = (0.60 - STACK_LATERAL_SEPARATION_M) / 2.0
LOW_STACK_HEIGHT_M = 0.45
HIGH_STACK_HEIGHT_M = 1.15


def _orbit_phase(step):
    """Create one synchronized orbit waypoint."""
    angle = 2.0 * math.pi * step / ORBIT_STEPS
    cos_angle = math.cos(angle)
    sin_angle = math.sin(angle)

    return {
        'name': f'ORBIT_{step:02d}',
        'hold_s': 0.0,
        'arrival_tolerance_m': ORBIT_ARRIVAL_TOLERANCE_M,
        'timeout_s': ORBIT_PHASE_TIMEOUT_S,
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
        'arrival_tolerance_m': STATIONARY_ARRIVAL_TOLERANCE_M,
        'timeout_s': STATIONARY_PHASE_TIMEOUT_S,
        'offsets': {
            'cf1': (0.0, 0.0, ORBIT_HEIGHT_M),
            'cf2': (0.0, 0.0, ORBIT_HEIGHT_M),
        },
    },
    {
        'name': 'VERTICAL_STACK',
        'hold_s': 3.0,
        'arrival_tolerance_m': STATIONARY_ARRIVAL_TOLERANCE_M,
        'timeout_s': STATIONARY_PHASE_TIMEOUT_S,
        'offsets': {
            # Homes are 0.60 m apart.  Moving each aircraft 0.075 m toward
            # center leaves 0.45 m lateral clearance while cf2 climbs above
            # cf1.  This is intentionally staggered instead of coaxial.
            'cf1': (STACK_INWARD_OFFSET_M, 0.0, LOW_STACK_HEIGHT_M),
            'cf2': (-STACK_INWARD_OFFSET_M, 0.0, HIGH_STACK_HEIGHT_M),
        },
    },
    {
        'name': 'UNSTACK',
        'hold_s': 2.0,
        'arrival_tolerance_m': STATIONARY_ARRIVAL_TOLERANCE_M,
        'timeout_s': STATIONARY_PHASE_TIMEOUT_S,
        'offsets': {
            'cf1': (0.0, 0.0, ORBIT_HEIGHT_M),
            'cf2': (0.0, 0.0, ORBIT_HEIGHT_M),
        },
    },
    # UNSTACK is already the step-zero orbit pose. Start at step one so the
    # coordinator does not immediately consume a duplicate waypoint.
    *[_orbit_phase(step) for step in range(1, ORBIT_STEPS + 1)],
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
