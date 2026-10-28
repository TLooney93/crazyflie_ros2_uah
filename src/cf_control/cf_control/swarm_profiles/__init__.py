"""Public API for named swarm vehicle profiles."""

from cf_control.swarm_profiles.loader import (
    load_swarm_profile,
    SWARM_PROFILES,
)

__all__ = ['SWARM_PROFILES', 'load_swarm_profile']
