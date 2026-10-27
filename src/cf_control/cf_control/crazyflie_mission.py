"""Editable autonomous missions for one Crazyflie.

All offsets are ``(x, y, z)`` in metres relative to the position recorded
when the vehicle is armed. Waypoint hold time starts only after the vehicle
is within the configured position tolerance.
"""

import copy
import math

from cf_control.config import (
    EXTERNAL_MAX_HEIGHT_M,
    EXTERNAL_MAX_HORIZONTAL_OFFSET_M,
    EXTERNAL_MIN_HEIGHT_M,
    FLIGHT_TEST_FIGURE_EIGHT_DURATION_S,
    FLIGHT_TEST_FIGURE_EIGHT_RADIUS_M,
    FLIGHT_TEST_HEIGHT_M,
    FLIGHT_TEST_HOVER_S,
)


CRAZYFLIE_MISSIONS = {
    'figure_eight': [
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
    ],
    'hover_test': [
        {
            'name': 'HOVER',
            'kind': 'waypoint',
            'offset': (0.0, 0.0, FLIGHT_TEST_HEIGHT_M),
            'hold_s': 5.0,
        },
    ],
    'square_demo': [
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
    ],
}


_RESERVED_PHASE_NAMES = {
    'TAKEOFF',
    'EXTERNAL_TAKEOFF',
    'EXTERNAL_HOVER',
    'WAND_TAKEOFF',
    'WAND_HOVER',
}


def _is_finite_number(value):
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
    )


def _validate_offset(offset, phase_name, field_name):
    if (
        not isinstance(offset, (tuple, list))
        or len(offset) != 3
        or not all(_is_finite_number(value) for value in offset)
    ):
        raise ValueError(
            f'Phase {phase_name!r} {field_name} must contain three '
            'finite numbers'
        )

    if math.hypot(offset[0], offset[1]) > (
        EXTERNAL_MAX_HORIZONTAL_OFFSET_M
    ):
        raise ValueError(
            f'Phase {phase_name!r} exceeds the horizontal target limit'
        )
    if not EXTERNAL_MIN_HEIGHT_M <= offset[2] <= EXTERNAL_MAX_HEIGHT_M:
        raise ValueError(
            f'Phase {phase_name!r} exceeds the height target limit'
        )


def load_crazyflie_mission(name):
    """Return an independently mutable, validated single-drone mission."""
    if name not in CRAZYFLIE_MISSIONS:
        choices = ', '.join(sorted(CRAZYFLIE_MISSIONS))
        raise ValueError(
            f'Unknown Crazyflie mission {name!r}; choose: {choices}'
        )

    phases = CRAZYFLIE_MISSIONS[name]
    if not isinstance(phases, (tuple, list)) or not phases:
        raise ValueError(f'Crazyflie mission {name!r} has no phases')

    phase_names = set()
    for phase in phases:
        phase_name = phase.get('name')
        if not isinstance(phase_name, str) or not phase_name.strip():
            raise ValueError('Every mission phase needs a non-empty name')
        if phase_name in phase_names:
            raise ValueError(f'Duplicate mission phase name {phase_name!r}')
        if phase_name in _RESERVED_PHASE_NAMES:
            raise ValueError(f'Mission phase name {phase_name!r} is reserved')
        phase_names.add(phase_name)

        kind = phase.get('kind')
        if kind == 'waypoint':
            offset = phase.get('offset')
            _validate_offset(offset, phase_name, 'offset')
            hold_s = phase.get('hold_s', 0.0)
            if not _is_finite_number(hold_s) or hold_s < 0.0:
                raise ValueError(
                    f'Phase {phase_name!r} hold_s must be finite and '
                    'non-negative'
                )
        elif kind == 'figure_eight':
            center = phase.get('center_offset')
            _validate_offset(center, phase_name, 'center_offset')
            radius_m = phase.get('radius_m')
            duration_s = phase.get('duration_s')
            if not _is_finite_number(radius_m) or radius_m <= 0.0:
                raise ValueError(
                    f'Phase {phase_name!r} radius_m must be positive'
                )
            if not _is_finite_number(duration_s) or duration_s <= 0.0:
                raise ValueError(
                    f'Phase {phase_name!r} duration_s must be positive'
                )
            if math.hypot(center[0], center[1]) + radius_m > (
                EXTERNAL_MAX_HORIZONTAL_OFFSET_M
            ):
                raise ValueError(
                    f'Phase {phase_name!r} trajectory exceeds the '
                    'horizontal target limit'
                )
        else:
            raise ValueError(
                f'Phase {phase_name!r} kind must be waypoint or '
                'figure_eight'
            )

    final_phase = phases[-1]
    final_offset = (
        final_phase['offset']
        if final_phase['kind'] == 'waypoint'
        else final_phase['center_offset']
    )
    if not math.isclose(final_offset[0], 0.0, abs_tol=1e-9) or not (
        math.isclose(final_offset[1], 0.0, abs_tol=1e-9)
    ):
        raise ValueError(
            f'Crazyflie mission {name!r} must finish above its arming '
            'position before landing'
        )

    return copy.deepcopy(phases)
