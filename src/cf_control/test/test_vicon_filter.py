"""Tests for time-domain Vicon position filtering."""

from cf_control.vicon_filter import ViconPositionFilter

import pytest


def test_brief_large_change_is_rejected_by_duration():
    """A short substitution must be rejected regardless of its distance."""
    position_filter = ViconPositionFilter(0.10)
    for received_at in (0.00, 0.02, 0.04, 0.06):
        position_filter.update((1.0, 2.0, 0.5), received_at)

    position_filter.update((20.0, -10.0, 4.0), 0.08)
    filtered = position_filter.update((1.0, 2.0, 0.5), 0.10)

    assert filtered == pytest.approx((1.0, 2.0, 0.5))


def test_brief_small_change_is_also_rejected_by_duration():
    """Filtering must not use a distance threshold."""
    position_filter = ViconPositionFilter(0.10)
    for received_at in (0.00, 0.02, 0.04, 0.06):
        position_filter.update((1.0, 2.0, 0.5), received_at)

    position_filter.update((1.01, 2.01, 0.51), 0.08)
    filtered = position_filter.update((1.0, 2.0, 0.5), 0.10)

    assert filtered == pytest.approx((1.0, 2.0, 0.5))


def test_sustained_change_is_accepted_after_half_window():
    """A real change must pass after occupying most of the time window."""
    position_filter = ViconPositionFilter(0.10)
    position_filter.update((0.0, 0.0, 0.5), 0.00)
    position_filter.update((0.0, 0.0, 0.5), 0.02)
    position_filter.update((2.0, 1.0, 0.8), 0.04)
    position_filter.update((2.0, 1.0, 0.8), 0.08)

    filtered = position_filter.update((2.0, 1.0, 0.8), 0.12)

    assert filtered == pytest.approx((2.0, 1.0, 0.8))


def test_filter_uses_elapsed_time_instead_of_sample_count():
    """Different callback rates must represent equal durations equally."""
    sparse_filter = ViconPositionFilter(0.10)
    dense_filter = ViconPositionFilter(0.10)

    sparse_filter.update((0.0, 0.0, 0.0), 0.00)
    sparse_filter.update((5.0, 5.0, 5.0), 0.08)
    sparse_result = sparse_filter.update((0.0, 0.0, 0.0), 0.10)

    for received_at in (0.00, 0.02, 0.04, 0.06):
        dense_filter.update((0.0, 0.0, 0.0), received_at)
    dense_filter.update((5.0, 5.0, 5.0), 0.08)
    dense_result = dense_filter.update((0.0, 0.0, 0.0), 0.10)

    assert sparse_result == pytest.approx((0.0, 0.0, 0.0))
    assert dense_result == pytest.approx(sparse_result)


def test_reset_discards_time_history():
    """Reset must let a new tracking session establish its own history."""
    position_filter = ViconPositionFilter(0.10)
    position_filter.update((0.0, 0.0, 0.0), 0.00)
    position_filter.update((0.0, 0.0, 0.0), 0.05)

    position_filter.reset()

    assert position_filter.update(
        (2.0, 1.0, 0.5),
        1.0,
    ) == (2.0, 1.0, 0.5)


@pytest.mark.parametrize('window_s', (0.0, -1.0, float('inf')))
def test_filter_requires_positive_finite_window(window_s):
    """Invalid time windows must fail during configuration."""
    with pytest.raises(ValueError):
        ViconPositionFilter(window_s)


def test_filter_rejects_non_monotonic_timestamps():
    """Out-of-order time must not corrupt duration weighting."""
    position_filter = ViconPositionFilter(0.10)
    position_filter.update((0.0, 0.0, 0.0), 1.0)

    with pytest.raises(ValueError):
        position_filter.update((0.0, 0.0, 0.0), 0.9)
