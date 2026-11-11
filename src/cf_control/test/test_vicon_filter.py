"""Tests for brief Vicon position-jump filtering."""

import pytest

from cf_control.vicon_filter import ViconPositionFilter


def test_single_sample_jump_does_not_move_filtered_position():
    """One bad optical frame must not enter the controller position."""
    position_filter = ViconPositionFilter(3)
    position_filter.update((1.0, 2.0, 0.5))
    position_filter.update((1.0, 2.0, 0.5))

    filtered = position_filter.update((1.25, 1.75, 0.75))

    assert filtered == pytest.approx((1.0, 2.0, 0.5))


def test_motion_continues_after_single_sample_jump():
    """A spike must not leave a lasting offset after tracking recovers."""
    position_filter = ViconPositionFilter(3)
    position_filter.update((0.00, 0.0, 0.5))
    position_filter.update((0.01, 0.0, 0.5))
    position_filter.update((0.30, 0.3, 0.8))

    filtered = position_filter.update((0.02, 0.0, 0.5))

    assert filtered == pytest.approx((0.02, 0.0, 0.5))


def test_sustained_motion_passes_with_one_sample_delay():
    """The median follows coherent motion instead of freezing position."""
    position_filter = ViconPositionFilter(3)

    outputs = [
        position_filter.update((x, 0.0, 0.5))[0]
        for x in (0.00, 0.01, 0.02, 0.03)
    ]

    assert outputs == pytest.approx((0.00, 0.005, 0.01, 0.02))


def test_reset_discards_old_tracking_cluster():
    """A deliberate disarmed reacquisition must not blend two locations."""
    position_filter = ViconPositionFilter(3)
    position_filter.update((0.0, 0.0, 0.0))
    position_filter.update((0.0, 0.0, 0.0))

    position_filter.reset((2.0, 1.0, 0.5))

    assert position_filter.update((2.0, 1.0, 0.5)) == (2.0, 1.0, 0.5)


@pytest.mark.parametrize('sample_count', (0, 2, -1))
def test_filter_requires_positive_odd_window(sample_count):
    """Invalid windows must fail during configuration."""
    with pytest.raises(ValueError):
        ViconPositionFilter(sample_count)
