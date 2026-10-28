"""Pure helpers shared by the wand input and flight-control nodes."""

import math

from cf_control.config import (
    VALID_WAND_GESTURES,
    WAND_GESTURE_ACTIONS,
    WAND_GESTURE_BASELINE_ALPHA,
    WAND_GESTURE_COOLDOWN_S,
    WAND_GESTURE_HORIZONTAL_TRAVEL_M,
    WAND_GESTURE_MAX_DURATION_S,
    WAND_GESTURE_RETURN_TOLERANCE_M,
    WAND_GESTURE_VERTICAL_TRAVEL_M,
    WAND_SIMULATION_STEP_M,
)


def normalize_gesture(value):
    """Return a normalized gesture name, or None when unsupported."""
    gesture = value.strip().lower().replace(' ', '_')
    return gesture if gesture in VALID_WAND_GESTURES else None


def gesture_action(value):
    """Return the controller action associated with a valid gesture."""
    gesture = normalize_gesture(value)
    if gesture is None:
        return None
    return WAND_GESTURE_ACTIONS.get(gesture, gesture)


class PositionGestureRecognizer:
    """Recognize deliberate three-part gestures from tracked XYZ positions."""

    def __init__(
        self,
        vertical_travel_m=WAND_GESTURE_VERTICAL_TRAVEL_M,
        horizontal_travel_m=WAND_GESTURE_HORIZONTAL_TRAVEL_M,
        return_tolerance_m=WAND_GESTURE_RETURN_TOLERANCE_M,
        max_duration_s=WAND_GESTURE_MAX_DURATION_S,
        cooldown_s=WAND_GESTURE_COOLDOWN_S,
        baseline_alpha=WAND_GESTURE_BASELINE_ALPHA,
    ):
        values = (
            vertical_travel_m,
            horizontal_travel_m,
            return_tolerance_m,
            max_duration_s,
            cooldown_s,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError('Gesture thresholds and timing must be positive')
        if not math.isfinite(baseline_alpha) or not 0.0 <= baseline_alpha <= 1.0:
            raise ValueError('baseline_alpha must be between zero and one')

        self.vertical_travel_m = vertical_travel_m
        self.horizontal_travel_m = horizontal_travel_m
        self.return_tolerance_m = return_tolerance_m
        self.max_duration_s = max_duration_s
        self.cooldown_s = cooldown_s
        self.baseline_alpha = baseline_alpha
        self.baseline = None
        self.state = 'IDLE'
        self.started_at = None
        self.cooldown_until = 0.0
        self.horizontal_direction = None
        self.last_sample_at = None

    def update(self, position, received_at):
        """Consume one tracked position and return a completed gesture name."""
        position = tuple(float(value) for value in position)
        received_at = float(received_at)
        if (
            len(position) != 3
            or not all(math.isfinite(value) for value in position)
            or not math.isfinite(received_at)
        ):
            raise ValueError('Gesture samples must contain finite XYZ and time')

        if (
            self.last_sample_at is not None
            and received_at < self.last_sample_at
        ):
            self._reset(position)
        self.last_sample_at = received_at

        if self.baseline is None:
            self.baseline = position
            return None

        if received_at < self.cooldown_until:
            self._update_baseline(position)
            return None

        if (
            self.state != 'IDLE'
            and received_at - self.started_at > self.max_duration_s
        ):
            self._reset(position)
            return None

        dx = position[0] - self.baseline[0]
        dy = position[1] - self.baseline[1]
        dz = position[2] - self.baseline[2]
        horizontal_distance = math.hypot(dx, dy)

        if self.state == 'IDLE':
            if (
                abs(dz) >= self.vertical_travel_m
                and abs(dz) >= horizontal_distance
            ):
                self.state = 'UP' if dz > 0.0 else 'DOWN'
                self.started_at = received_at
                return None
            if horizontal_distance >= self.horizontal_travel_m:
                self.state = 'HORIZONTAL'
                self.started_at = received_at
                self.horizontal_direction = (
                    dx / horizontal_distance,
                    dy / horizontal_distance,
                )
                return None
            self._update_baseline(position)
            return None

        if self.state == 'UP' and abs(dz) <= self.return_tolerance_m:
            return self._complete('up_down', position, received_at)
        if self.state == 'DOWN' and abs(dz) <= self.return_tolerance_m:
            return self._complete('down_up', position, received_at)
        if self.state == 'HORIZONTAL':
            direction_x, direction_y = self.horizontal_direction
            projection = dx * direction_x + dy * direction_y
            if projection <= -self.horizontal_travel_m:
                return self._complete(
                    'side_to_side',
                    position,
                    received_at,
                )
        return None

    def _complete(self, gesture, position, received_at):
        self._reset(position)
        self.cooldown_until = received_at + self.cooldown_s
        return gesture

    def _reset(self, position):
        self.baseline = position
        self.state = 'IDLE'
        self.started_at = None
        self.horizontal_direction = None

    def _update_baseline(self, position):
        alpha = self.baseline_alpha
        self.baseline = tuple(
            previous + alpha * (current - previous)
            for previous, current in zip(self.baseline, position)
        )


def move_simulated_position(
    position,
    gesture,
    step=WAND_SIMULATION_STEP_M,
):
    """Return a simulated position after one directional gesture."""
    x, y, z = position
    offsets = {
        'move_forward': (step, 0.0, 0.0),
        'move_backward': (-step, 0.0, 0.0),
        'move_left': (0.0, step, 0.0),
        'move_right': (0.0, -step, 0.0),
    }
    dx, dy, dz = offsets.get(gesture, (0.0, 0.0, 0.0))
    return (x + dx, y + dy, z + dz)


def euler_degrees_to_quaternion(roll, pitch, yaw):
    """Convert Euler angles in degrees to an x-y-z-w quaternion."""
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)

    cy = math.cos(yaw / 2.0)
    sy = math.sin(yaw / 2.0)
    cp = math.cos(pitch / 2.0)
    sp = math.sin(pitch / 2.0)
    cr = math.cos(roll / 2.0)
    sr = math.sin(roll / 2.0)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quaternion_to_euler_degrees(quaternion):
    """Convert an x-y-z-w quaternion to roll, pitch, and yaw degrees."""
    x, y, z, w = _normalized(quaternion)

    roll = math.atan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y),
    )
    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    return tuple(math.degrees(value) for value in (roll, pitch, yaw))


def tilt_to_unit_offset(reference, current, threshold_deg):
    """Map the strongest relative wand tilt to a world-frame XY direction."""
    reference = _normalized(reference)
    current = _normalized(current)
    relative = _multiply(_conjugate(reference), current)
    roll, pitch, _ = quaternion_to_euler_degrees(relative)

    if max(abs(roll), abs(pitch)) < threshold_deg:
        return None

    _, _, reference_yaw = quaternion_to_euler_degrees(reference)
    yaw = math.radians(reference_yaw)

    if abs(pitch) >= abs(roll):
        forward = -math.copysign(1.0, pitch)
        left = 0.0
    else:
        forward = 0.0
        left = -math.copysign(1.0, roll)

    dx = forward * math.cos(yaw) - left * math.sin(yaw)
    dy = forward * math.sin(yaw) + left * math.cos(yaw)
    return dx, dy


def _normalized(quaternion):
    values = tuple(float(value) for value in quaternion)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Quaternion must contain finite values')
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1e-9:
        raise ValueError('Quaternion must have non-zero length')
    return tuple(value / norm for value in values)


def _conjugate(quaternion):
    x, y, z, w = quaternion
    return -x, -y, -z, w


def _multiply(first, second):
    ax, ay, az, aw = first
    bx, by, bz, bw = second
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )
