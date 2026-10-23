"""Validation helpers for Vicon pose measurements."""

import math
from collections import deque


def normalize_quaternion(quaternion):
    """Return a normalized quaternion, or None for invalid input."""
    if len(quaternion) != 4:
        return None
    if not all(math.isfinite(value) for value in quaternion):
        return None
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm < 1.0e-9:
        return None
    return tuple(value / norm for value in quaternion)


class ViconPoseValidator:
    """Track freshness and stationary stability of external poses."""

    def __init__(self, sample_count, timeout_s, max_spread_m):
        if sample_count < 2:
            raise ValueError('sample_count must be at least 2')
        if timeout_s <= 0.0 or max_spread_m <= 0.0:
            raise ValueError('timeout and spread must be positive')
        self.timeout_s = timeout_s
        self.max_spread_m = max_spread_m
        self.positions = deque(maxlen=sample_count)
        self.last_received_at = None
        self.latest_pose = None

    def add(self, position, quaternion, received_at):
        """Validate and store one pose, returning the normalized pose."""
        if not all(math.isfinite(value) for value in position):
            return None
        normalized = normalize_quaternion(quaternion)
        if normalized is None or not math.isfinite(received_at):
            return None
        if (
            self.last_received_at is not None
            and received_at - self.last_received_at > self.timeout_s
        ):
            self.positions.clear()
        position = tuple(float(value) for value in position)
        self.positions.append(position)
        self.last_received_at = received_at
        self.latest_pose = (position, normalized)
        return self.latest_pose

    def is_fresh(self, now):
        """Return True when the last pose is recent enough."""
        return (
            self.last_received_at is not None
            and now - self.last_received_at <= self.timeout_s
        )

    def is_ready(self, now):
        """Return True after enough fresh, spatially stable samples."""
        if not self.is_fresh(now) or len(self.positions) < self.positions.maxlen:
            return False
        for axis in range(3):
            values = [position[axis] for position in self.positions]
            if max(values) - min(values) > self.max_spread_m:
                return False
        return True

    def reset(self):
        """Forget all samples and require stability to be established again."""
        self.positions.clear()
        self.last_received_at = None
        self.latest_pose = None
