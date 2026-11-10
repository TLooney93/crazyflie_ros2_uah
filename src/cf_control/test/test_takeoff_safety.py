"""Tests for conservative heavy-airframe takeoff aborts."""

from cf_control.config import (
    TAKEOFF_ASSIST_GRACE_S,
    TAKEOFF_GRACE_MAX_VERTICAL_VELOCITY_M_S,
    TAKEOFF_MAX_VERTICAL_VELOCITY_M_S,
    TAKEOFF_PROGRESS_TIMEOUT_S,
    TAKEOFF_THRUST_SATURATION_DURATION_S,
    TAKEOFF_THRUST_SATURATION_THRESHOLD,
)
from cf_control.takeoff import (
    TakeoffSafetyLimits,
    TakeoffSafetyMonitor,
    takeoff_vertical_velocity_limit,
)


def _monitor():
    return TakeoffSafetyMonitor(
        TakeoffSafetyLimits(
            max_horizontal_deviation_m=0.30,
            assist_grace_s=TAKEOFF_ASSIST_GRACE_S,
            min_progress_height_m=0.05,
            progress_timeout_s=TAKEOFF_PROGRESS_TIMEOUT_S,
            thrust_saturation_threshold=(
                TAKEOFF_THRUST_SATURATION_THRESHOLD
            ),
            thrust_saturation_duration_s=(
                TAKEOFF_THRUST_SATURATION_DURATION_S
            ),
        )
    )


def _check(monitor, position, now, thrust=0.0):
    return monitor.check(
        position=position,
        home_position=(1.0, 2.0, 0.05),
        now=now,
        flight_started_at=10.0,
        stabilizer_thrust=thrust,
    )


def test_takeoff_aborts_without_minimum_vertical_progress():
    """A vehicle that remains near the floor must not keep accelerating."""
    reason = _check(
        _monitor(),
        (1.0, 2.0, 0.08),
        10.0 + TAKEOFF_ASSIST_GRACE_S + TAKEOFF_PROGRESS_TIMEOUT_S,
    )

    assert 'insufficient vertical progress' in reason


def test_takeoff_aborts_after_sustained_high_thrust_without_climb():
    """Sustained high thrust near the floor must stop the takeoff early."""
    monitor = _monitor()
    position = (1.0, 2.0, 0.08)

    first_reason = _check(
        monitor,
        position,
        10.0 + TAKEOFF_ASSIST_GRACE_S,
        TAKEOFF_THRUST_SATURATION_THRESHOLD,
    )
    final_reason = _check(
        monitor,
        position,
        10.0
        + TAKEOFF_ASSIST_GRACE_S
        + TAKEOFF_THRUST_SATURATION_DURATION_S,
        TAKEOFF_THRUST_SATURATION_THRESHOLD,
    )

    assert first_reason is None
    assert 'thrust remained saturated' in final_reason


def test_takeoff_saturation_timer_clears_after_vertical_progress():
    """Normal climbing must clear a transient high-thrust observation."""
    monitor = _monitor()
    _check(
        monitor,
        (1.0, 2.0, 0.08),
        10.0 + TAKEOFF_ASSIST_GRACE_S + 0.5,
        TAKEOFF_THRUST_SATURATION_THRESHOLD,
    )

    reason = _check(
        monitor,
        (1.0, 2.0, 0.11),
        10.0 + TAKEOFF_ASSIST_GRACE_S + 0.7,
        TAKEOFF_THRUST_SATURATION_THRESHOLD,
    )

    assert reason is None
    assert monitor.thrust_saturation_started_at is None


def test_assist_grace_delays_noncritical_stall_filters():
    """A heavy vehicle gets the full grace window before stall timing."""
    monitor = _monitor()
    reason = _check(
        monitor,
        (1.0, 2.0, 0.05),
        10.0 + TAKEOFF_ASSIST_GRACE_S - 0.01,
        TAKEOFF_THRUST_SATURATION_THRESHOLD,
    )

    assert reason is None
    assert monitor.thrust_saturation_started_at is None


def test_assist_grace_keeps_hard_launch_column_limit():
    """Launch assistance must not permit a horizontal flyaway."""
    reason = _check(
        _monitor(),
        (1.31, 2.0, 0.05),
        10.1,
        TAKEOFF_THRUST_SATURATION_THRESHOLD,
    )

    assert 'left launch column' in reason


def test_assist_grace_allows_full_boost_then_restores_normal_cap():
    assert takeoff_vertical_velocity_limit(
        TAKEOFF_ASSIST_GRACE_S - 0.01,
        TAKEOFF_ASSIST_GRACE_S,
        TAKEOFF_GRACE_MAX_VERTICAL_VELOCITY_M_S,
        TAKEOFF_MAX_VERTICAL_VELOCITY_M_S,
    ) == TAKEOFF_GRACE_MAX_VERTICAL_VELOCITY_M_S
    assert takeoff_vertical_velocity_limit(
        TAKEOFF_ASSIST_GRACE_S,
        TAKEOFF_ASSIST_GRACE_S,
        TAKEOFF_GRACE_MAX_VERTICAL_VELOCITY_M_S,
        TAKEOFF_MAX_VERTICAL_VELOCITY_M_S,
    ) == TAKEOFF_MAX_VERTICAL_VELOCITY_M_S
