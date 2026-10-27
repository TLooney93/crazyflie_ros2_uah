"""Tests for wand gesture and orientation helpers."""

import math

from cf_control.wand_control import (
    euler_degrees_to_quaternion,
    move_simulated_position,
    normalize_gesture,
    tilt_to_unit_offset,
)


def test_normalize_gesture_accepts_only_supported_input():
    assert normalize_gesture(' Move Left ') == 'move_left'
    assert normalize_gesture('TAKEOFF') == 'takeoff'
    assert normalize_gesture('flip') is None


def test_simulated_movement_uses_world_axes():
    assert move_simulated_position(
        (1.0, 2.0, 0.5),
        'move_forward',
    ) == (1.05, 2.0, 0.5)


def test_zero_rotation_is_identity_quaternion():
    assert euler_degrees_to_quaternion(0.0, 0.0, 0.0) == (
        0.0,
        0.0,
        0.0,
        1.0,
    )


def test_forward_tilt_maps_to_reference_heading():
    reference = euler_degrees_to_quaternion(0.0, 0.0, 90.0)
    current = euler_degrees_to_quaternion(0.0, -30.0, 90.0)
    dx, dy = tilt_to_unit_offset(reference, current, 20.0)

    assert math.isclose(dx, 0.0, abs_tol=1e-9)
    assert math.isclose(dy, 1.0, abs_tol=1e-9)


def test_small_tilt_is_inside_dead_zone():
    reference = euler_degrees_to_quaternion(0.0, 0.0, 0.0)
    current = euler_degrees_to_quaternion(5.0, -10.0, 0.0)

    assert tilt_to_unit_offset(reference, current, 20.0) is None
