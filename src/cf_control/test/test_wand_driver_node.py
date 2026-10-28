"""Tests for the physical wand driver helpers."""

import math

from cf_control.wand_driver_node import euler_degrees_to_quaternion


def test_zero_rotation_is_identity_quaternion():
    quaternion = euler_degrees_to_quaternion(0.0, 0.0, 0.0)

    assert quaternion == (0.0, 0.0, 0.0, 1.0)


def test_ninety_degree_yaw():
    x, y, z, w = euler_degrees_to_quaternion(0.0, 0.0, 90.0)

    assert math.isclose(x, 0.0, abs_tol=1e-9)
    assert math.isclose(y, 0.0, abs_tol=1e-9)
    assert math.isclose(z, math.sqrt(0.5), abs_tol=1e-9)
    assert math.isclose(w, math.sqrt(0.5), abs_tol=1e-9)