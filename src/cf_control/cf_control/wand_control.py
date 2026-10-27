"""Pure helpers shared by the wand input and flight-control nodes."""

import math

from cf_control.config import VALID_WAND_GESTURES, WAND_SIMULATION_STEP_M


def normalize_gesture(value):
    """Return a normalized gesture name, or None when unsupported."""
    gesture = value.strip().lower().replace(' ', '_')
    return gesture if gesture in VALID_WAND_GESTURES else None


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
