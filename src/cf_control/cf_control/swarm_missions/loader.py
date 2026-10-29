"""
Registry and validation for saved swarm missions.

Offsets are meters relative to each drone's position when it was armed. To
register a mission, import its ``MISSION_NAME`` and ``MISSION`` below and add
that pair to ``SWARM_MISSIONS``.
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
        offsets = phase.get('offsets', {})
        if set(offsets) != expected_ids:
            raise ValueError(
                f'Phase {phase_name!r} offsets must exactly match '
                f'{sorted(expected_ids)}'
            )

        hold_s = phase.get('hold_s')
        if not isinstance(hold_s, (int, float)) or not math.isfinite(hold_s):
            raise ValueError(f'Phase {phase_name!r} hold_s must be finite')
        if hold_s < 0.0:
            raise ValueError(f'Phase {phase_name!r} hold_s cannot be negative')

        for drone_id, offset in offsets.items():
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
