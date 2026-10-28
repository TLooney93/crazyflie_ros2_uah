"""
Registry and validation for named swarm vehicle profiles.

To register a profile, import its ``PROFILE_NAME`` and ``PROFILE`` below and
add that pair to ``SWARM_PROFILES``.
"""

import copy

from cf_control.swarm_profiles.five_drone_lab import (
    PROFILE as FIVE_DRONE_LAB,
    PROFILE_NAME as FIVE_DRONE_LAB_NAME,
)
from cf_control.swarm_profiles.two_drone_lab import (
    PROFILE as TWO_DRONE_LAB,
    PROFILE_NAME as TWO_DRONE_LAB_NAME,
)


SWARM_PROFILES = {
    TWO_DRONE_LAB_NAME: TWO_DRONE_LAB,
    FIVE_DRONE_LAB_NAME: FIVE_DRONE_LAB,
}


def load_swarm_profile(name, require_radio_uris=False):
    """Return a validated, independently mutable swarm profile."""
    if name not in SWARM_PROFILES:
        choices = ', '.join(sorted(SWARM_PROFILES))
        raise ValueError(f'Unknown swarm profile {name!r}; choose: {choices}')

    profile = copy.deepcopy(SWARM_PROFILES[name])
    drone_ids = profile.get('drone_ids')
    if not isinstance(drone_ids, (tuple, list)) or len(drone_ids) < 2:
        raise ValueError(
            f'Swarm profile {name!r} needs at least two drone_ids'
        )
    if any(not isinstance(item, str) or not item for item in drone_ids):
        raise ValueError(f'Swarm profile {name!r} has an invalid drone_id')
    if len(set(drone_ids)) != len(drone_ids):
        raise ValueError(f'Swarm profile {name!r} has duplicate drone_ids')

    expected_ids = set(drone_ids)
    for field in ('vicon_topics', 'radio_uris'):
        values = profile.get(field)
        if not isinstance(values, dict) or set(values) != expected_ids:
            raise ValueError(
                f'Swarm profile {name!r} {field} must exactly match '
                f'{sorted(expected_ids)}'
            )
        if any(not isinstance(value, str) for value in values.values()):
            raise ValueError(
                f'Swarm profile {name!r} {field} values must be strings'
            )

    if any(
        not topic.startswith('/')
        for topic in profile['vicon_topics'].values()
    ):
        raise ValueError(
            f'Swarm profile {name!r} Vicon topics must be absolute'
        )

    if require_radio_uris:
        missing = [
            drone_id
            for drone_id in drone_ids
            if not profile['radio_uris'][drone_id]
        ]
        if missing:
            raise ValueError(
                f'Swarm profile {name!r} is missing radio URIs for '
                f'{missing}; fill them in swarm_profiles/{name}.py'
            )

    mission_name = profile.get('mission_name')
    if not isinstance(mission_name, str) or not mission_name:
        raise ValueError(
            f'Swarm profile {name!r} needs a default mission_name'
        )

    world = profile.get('simulation_world')
    if (
        not isinstance(world, str)
        or not world.endswith('.sdf')
        or '/' in world
    ):
        raise ValueError(
            f'Swarm profile {name!r} has an invalid simulation_world'
        )

    profile['drone_ids'] = list(drone_ids)
    return profile
