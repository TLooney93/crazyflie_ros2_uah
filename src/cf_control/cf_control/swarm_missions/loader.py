"""
Registry and validation for saved swarm missions.

``offsets`` are meters relative to each drone's arming position.
``formation_targets`` are along/across coordinates relative to the measured
midpoint and cf1-to-cf2 launch baseline. ``trajectory`` defines a continuous
two-drone formation path. To register a mission, import its ``MISSION_NAME``
and ``MISSION`` below and add that pair to ``SWARM_MISSIONS``.
"""

import math

from cf_control.config import (
    EXTERNAL_MAX_HEIGHT_M,
    EXTERNAL_MAX_HORIZONTAL_OFFSET_M,
    EXTERNAL_MIN_HEIGHT_M,
)
from cf_control.swarm_missions.five_drone_demo import (
    MISSION as FIVE_DRONE_DEMO,
    MISSION_NAME as FIVE_DRONE_DEMO_NAME,
)
from cf_control.swarm_missions.two_drone_demo import (
    MISSION as TWO_DRONE_DEMO,
    MISSION_NAME as TWO_DRONE_DEMO_NAME,
)
from cf_control.swarm_missions.two_drone_lab import (
    MISSION as TWO_DRONE_LAB,
    MISSION_NAME as TWO_DRONE_LAB_NAME,
)


SWARM_MISSIONS = {
    TWO_DRONE_LAB_NAME: TWO_DRONE_LAB,
    TWO_DRONE_DEMO_NAME: TWO_DRONE_DEMO,
    FIVE_DRONE_DEMO_NAME: FIVE_DRONE_DEMO,
}


def load_swarm_mission(name, drone_ids):
    """Return a validated mission for exactly ``drone_ids``."""
    if name not in SWARM_MISSIONS:
        choices = ', '.join(sorted(SWARM_MISSIONS))
        raise ValueError(f'Unknown swarm mission {name!r}; choose: {choices}')

    requested_ids = list(drone_ids)
    if not requested_ids or len(set(requested_ids)) != len(requested_ids):
        raise ValueError('drone_ids must be a non-empty list of unique names')

    phases = SWARM_MISSIONS[name]
    if not phases:
        raise ValueError(f'Swarm mission {name!r} has no phases')

    expected_ids = set(requested_ids)
    for phase in phases:
        phase_name = phase.get('name', '<unnamed>')
        offsets = phase.get('offsets')
        formation_targets = phase.get('formation_targets')
        trajectory = phase.get('trajectory')
        target_fields = [
            value
            for value in (offsets, formation_targets, trajectory)
            if value is not None
        ]
        if len(target_fields) != 1:
            raise ValueError(
                f'Phase {phase_name!r} must define exactly one of offsets, '
                'formation_targets, or trajectory'
            )
        coordinates = offsets if offsets is not None else formation_targets
        coordinate_name = (
            'offsets' if offsets is not None else 'formation_targets'
        )
        if coordinates is not None and set(coordinates) != expected_ids:
            raise ValueError(
                f'Phase {phase_name!r} {coordinate_name} must exactly match '
                f'{sorted(expected_ids)}'
            )
        if (
            formation_targets is not None or trajectory is not None
        ) and len(requested_ids) != 2:
            raise ValueError(
                'formation targets and trajectories require exactly two drones'
            )

        hold_s = phase.get('hold_s')
        if not isinstance(hold_s, (int, float)) or not math.isfinite(hold_s):
            raise ValueError(f'Phase {phase_name!r} hold_s must be finite')
        if hold_s < 0.0:
            raise ValueError(f'Phase {phase_name!r} hold_s cannot be negative')

        timeout_s = phase.get('timeout_s')
        if timeout_s is not None:
            if (
                not isinstance(timeout_s, (int, float))
                or not math.isfinite(timeout_s)
                or timeout_s <= hold_s
            ):
                raise ValueError(
                    f'Phase {phase_name!r} timeout_s must be finite and '
                    'greater than hold_s'
                )

        arrival_tolerance_m = phase.get('arrival_tolerance_m')
        if arrival_tolerance_m is not None:
            if (
                not isinstance(arrival_tolerance_m, (int, float))
                or not math.isfinite(arrival_tolerance_m)
                or arrival_tolerance_m <= 0.0
            ):
                raise ValueError(
                    f'Phase {phase_name!r} arrival_tolerance_m must be a '
                    'positive finite number'
                )

        if trajectory is not None:
            if trajectory.get('type') != 'formation_orbit':
                raise ValueError(
                    f'Phase {phase_name!r} has an unsupported trajectory'
                )
            radius_m = trajectory.get('radius_m')
            height_m = trajectory.get('height_m')
            duration_s = trajectory.get('duration_s')
            if not all(
                isinstance(value, (int, float)) and math.isfinite(value)
                for value in (radius_m, height_m, duration_s)
            ):
                raise ValueError(
                    f'Phase {phase_name!r} trajectory values must be finite'
                )
            if radius_m <= 0.0 or radius_m > EXTERNAL_MAX_HORIZONTAL_OFFSET_M:
                raise ValueError(
                    f'Phase {phase_name!r} has an invalid trajectory radius'
                )
            if not EXTERNAL_MIN_HEIGHT_M <= height_m <= EXTERNAL_MAX_HEIGHT_M:
                raise ValueError(
                    f'Phase {phase_name!r} has an invalid trajectory height'
                )
            if duration_s <= 0.0:
                raise ValueError(
                    f'Phase {phase_name!r} trajectory duration must be positive'
                )
            continue

        for drone_id, offset in coordinates.items():
            if len(offset) != 3 or not all(math.isfinite(v) for v in offset):
                raise ValueError(
                    f'Phase {phase_name!r} has an invalid offset for '
                    f'{drone_id}'
                )
            if math.hypot(offset[0], offset[1]) > (
                EXTERNAL_MAX_HORIZONTAL_OFFSET_M
            ):
                raise ValueError(
                    f'Phase {phase_name!r} exceeds the horizontal target '
                    f'limit for {drone_id}'
                )
            if not EXTERNAL_MIN_HEIGHT_M <= offset[2] <= EXTERNAL_MAX_HEIGHT_M:
                raise ValueError(
                    f'Phase {phase_name!r} exceeds the height target limit '
                    f'for {drone_id}'
                )

    return phases
