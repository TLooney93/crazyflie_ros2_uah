"""Small monotonic-time helpers for limiting high-rate input forwarding."""

import math


class SampleRateLimiter:
    """Accept at most one sample during each configured time interval."""

    def __init__(self, rate_hz):
        rate_hz = float(rate_hz)
        if not math.isfinite(rate_hz) or rate_hz <= 0.0:
            raise ValueError('rate_hz must be a positive finite number')
        self.rate_hz = rate_hz
        self.period_s = 1.0 / rate_hz
        self.last_accepted_at = None

    def accept(self, received_at):
        """Return whether a sample should be forwarded at this time."""
        received_at = float(received_at)
        if not math.isfinite(received_at):
            raise ValueError('received_at must be finite')
        if (
            self.last_accepted_at is None
            or received_at < self.last_accepted_at
            or received_at - self.last_accepted_at >= self.period_s
        ):
            self.last_accepted_at = received_at
            return True
        return False
