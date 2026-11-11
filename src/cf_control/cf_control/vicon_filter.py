"""Small, deterministic filters for Vicon position measurements."""

from collections import deque
from statistics import median


class ViconPositionFilter:
    """Reject isolated coordinate spikes with a short running median."""

    def __init__(self, sample_count):
        """Create an odd-width position median filter."""
        if sample_count < 1 or sample_count % 2 == 0:
            raise ValueError('sample_count must be a positive odd integer')
        self._positions = deque(maxlen=sample_count)

    def update(self, position):
        """Add a valid raw position and return its coordinate-wise median."""
        self._positions.append(tuple(position))
        return tuple(
            float(median(coordinates))
            for coordinates in zip(*self._positions)
        )

    def reset(self, position=None):
        """Discard history, optionally seeding it with a known position."""
        self._positions.clear()
        if position is not None:
            self._positions.append(tuple(position))
