"""Time-domain filtering for Vicon position measurements."""

from collections import deque
import math


class ViconPositionFilter:
    """Reject position changes that occupy less than half a time window."""

    def __init__(self, window_s):
        """Create a time-weighted coordinate median filter."""
        if not math.isfinite(window_s) or window_s <= 0.0:
            raise ValueError('window_s must be positive and finite')
        self.window_s = float(window_s)
        self._samples = deque()

    def update(self, position, received_at):
        """Add a position and return its time-weighted coordinate median."""
        if not math.isfinite(received_at):
            raise ValueError('received_at must be finite')
        if self._samples and received_at < self._samples[-1][0]:
            raise ValueError('received_at must be monotonic')

        sample = (float(received_at), tuple(position))
        self._samples.append(sample)
        cutoff = received_at - self.window_s

        # Retain the last sample at or before the cutoff because it defines
        # the position at the beginning of the time window.
        while len(self._samples) >= 2 and self._samples[1][0] <= cutoff:
            self._samples.popleft()

        if len(self._samples) == 1:
            return self._samples[0][1]

        weighted_positions = []
        for index in range(len(self._samples) - 1):
            sample_at, sample_position = self._samples[index]
            next_sample_at = self._samples[index + 1][0]
            duration_s = next_sample_at - max(sample_at, cutoff)
            if duration_s > 0.0:
                weighted_positions.append((sample_position, duration_s))

        if not weighted_positions:
            return self._samples[-1][1]

        return tuple(
            self._weighted_median(
                (position[axis], duration_s)
                for position, duration_s in weighted_positions
            )
            for axis in range(3)
        )

    def reset(self):
        """Discard all time history."""
        self._samples.clear()

    @staticmethod
    def _weighted_median(weighted_values):
        """Return a deterministic median weighted by sample duration."""
        values = sorted(weighted_values)
        total_weight = sum(weight for _value, weight in values)
        midpoint = total_weight / 2.0
        accumulated = 0.0
        for index, (value, weight) in enumerate(values):
            accumulated += weight
            if accumulated > midpoint:
                return float(value)
            if accumulated == midpoint and index + 1 < len(values):
                return float((value + values[index + 1][0]) / 2.0)
        return float(values[-1][0])
