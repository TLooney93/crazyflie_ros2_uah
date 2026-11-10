"""Takeoff command shaping and safety monitoring.

Both single-drone and swarm hardware flights use :class:`CrazyflieNode`, so
this module is the single source of truth for their takeoff behavior.  The
functions shape commands close to the floor; ``TakeoffSafetyMonitor`` detects
a vehicle that leaves its launch column or demands high thrust without
climbing.
"""

import math
from dataclasses import dataclass


def ground_effect_takeoff_boost(
    height_above_floor_m,
    boost_end_height_m,
    boost_m_s,
):
    """Return the vertical boost used below ``boost_end_height_m``."""
    if height_above_floor_m < boost_end_height_m:
        return boost_m_s
    return 0.0


def limit_horizontal_velocity(vx, vy, max_velocity):
    """Limit horizontal speed while preserving the requested direction."""
    speed = math.hypot(vx, vy)
    if speed <= max_velocity or speed == 0.0:
        return vx, vy
    scale = max_velocity / speed
    return vx * scale, vy * scale


def takeoff_horizontal_velocity_limit(
    takeoff_elapsed_s,
    grace_period_s,
    grace_limit_m_s,
    height_above_floor_m,
    near_floor_height_m,
    near_floor_limit_m_s,
    airborne_limit_m_s,
):
    """Return the horizontal speed limit for the current takeoff height."""
    if takeoff_elapsed_s < grace_period_s:
        return grace_limit_m_s
    if height_above_floor_m < near_floor_height_m:
        return near_floor_limit_m_s
    return airborne_limit_m_s


def takeoff_vertical_velocity_limit(
    takeoff_elapsed_s,
    grace_period_s,
    grace_limit_m_s,
    normal_limit_m_s,
):
    """Return the higher vertical cap only during launch assistance."""
    if takeoff_elapsed_s < grace_period_s:
        return grace_limit_m_s
    return normal_limit_m_s


@dataclass(frozen=True)
class TakeoffSafetyLimits:
    """Thresholds used to reject an unsafe or stalled takeoff."""

    max_horizontal_deviation_m: float
    assist_grace_s: float
    min_progress_height_m: float
    progress_timeout_s: float
    thrust_saturation_threshold: float
    thrust_saturation_duration_s: float


class TakeoffSafetyMonitor:
    """Track state needed to detect a stalled takeoff."""

    def __init__(self, limits):
        """Create a monitor using immutable takeoff safety limits."""
        self.limits = limits
        self.thrust_saturation_started_at = None

    def reset(self):
        """Clear observations from a previous takeoff."""
        self.thrust_saturation_started_at = None

    def check(
        self,
        *,
        position,
        home_position,
        now,
        flight_started_at,
        stabilizer_thrust,
    ):
        """Return an abort reason, or ``None`` while takeoff remains safe."""
        horizontal_deviation_m = math.hypot(
            position[0] - home_position[0],
            position[1] - home_position[1],
        )
        if horizontal_deviation_m > self.limits.max_horizontal_deviation_m:
            return (
                'Takeoff left launch column by '
                f'{horizontal_deviation_m:.3f} m'
            )

        height_above_floor_m = position[2] - home_position[2]
        takeoff_elapsed_s = (
            now - flight_started_at
            if flight_started_at is not None
            else 0.0
        )

        # The hard launch-column boundary above remains active. Only the
        # filters that can falsely classify a heavy, slowly accelerating
        # airframe as stalled are delayed during launch assistance.
        if takeoff_elapsed_s < self.limits.assist_grace_s:
            self.reset()
            return None

        monitored_elapsed_s = (
            takeoff_elapsed_s - self.limits.assist_grace_s
        )
        if (
            monitored_elapsed_s >= self.limits.progress_timeout_s
            and height_above_floor_m < self.limits.min_progress_height_m
        ):
            return (
                'Takeoff made insufficient vertical progress: '
                f'{height_above_floor_m:.3f} m after '
                f'{takeoff_elapsed_s:.1f} s '
                f'(including {self.limits.assist_grace_s:.1f} s assist)'
            )

        thrust_is_saturated = (
            stabilizer_thrust is not None
            and stabilizer_thrust
            >= self.limits.thrust_saturation_threshold
            and height_above_floor_m < self.limits.min_progress_height_m
        )
        if not thrust_is_saturated:
            self.reset()
            return None
        if self.thrust_saturation_started_at is None:
            self.thrust_saturation_started_at = now
            return None

        saturation_duration_s = now - self.thrust_saturation_started_at
        if saturation_duration_s >= self.limits.thrust_saturation_duration_s:
            return (
                'Takeoff thrust remained saturated without climbing: '
                f'{stabilizer_thrust:.0f} thrust for '
                f'{saturation_duration_s:.1f} s'
            )
        return None
