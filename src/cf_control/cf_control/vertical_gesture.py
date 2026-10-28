"""Hardware-independent, one-command-per-button-hold vertical gesture detector."""

import math


class VerticalGesture:
    """Recognize displacement in world Z; require release before another command."""

    def __init__(self, distance=0.20, timeout=0.25, samples=3):
        if not math.isfinite(distance) or distance <= 0:
            raise ValueError('distance must be finite and positive')
        if not math.isfinite(timeout) or timeout <= 0 or samples < 1:
            raise ValueError('timeout and sample count must be positive')
        self.distance, self.timeout, self.samples = distance, timeout, samples
        self.enabled = False
        self.reset()

    def reset(self):
        self.origin = None
        self.last_time = None
        self.candidate = None
        self.count = 0
        self.fired = False

    def control(self, gesture):
        if gesture == 'enable' and not self.enabled:
            self.reset()
            self.enabled = True
        elif gesture in ('disable', 'emergency_stop'):
            self.enabled = False
            self.reset()

    def expire(self, now):
        if self.last_time is not None and now - self.last_time > self.timeout:
            self.control('disable')

    def update(self, position, now):
        self.expire(now)
        if not self.enabled or self.fired:
            return None
        if not all(math.isfinite(v) for v in (*position, now)):
            self.control('disable')
            return None
        if self.last_time is not None and now <= self.last_time:
            self.control('disable')
            return None
        self.last_time = now
        if self.origin is None:
            self.origin = position
            return None
        dz = position[2] - self.origin[2]
        direction = 'takeoff' if dz >= self.distance else (
            'land' if dz <= -self.distance else None)
        if direction != self.candidate:
            self.count = 0
        self.candidate = direction
        self.count = self.count + 1 if direction else 0
        if self.count >= self.samples:
            self.fired = True
            return direction
        return None
