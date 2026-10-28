"""Tests for Vicon pose validation."""

import math

from cf_control.vicon import ViconPoseValidator, normalize_quaternion


def test_quaternion_is_normalized():
    assert normalize_quaternion((0.0, 0.0, 0.0, 2.0)) == (
        0.0, 0.0, 0.0, 1.0
    )


def test_invalid_quaternion_is_rejected():
    assert normalize_quaternion((0.0, 0.0, 0.0, 0.0)) is None
    assert normalize_quaternion((math.nan, 0.0, 0.0, 1.0)) is None


def test_validator_requires_stable_fresh_samples():
    validator = ViconPoseValidator(
        sample_count=3, timeout_s=0.25, max_spread_m=0.05
    )
    for index in range(3):
        pose = validator.add(
            (index * 0.01, 0.0, 0.5),
            (0.0, 0.0, 0.0, 1.0),
            index * 0.02,
        )
        assert pose is not None
    assert validator.is_ready(0.05) is True
    assert validator.is_fresh(0.29) is True
    assert validator.is_fresh(0.30) is False


def test_validator_rejects_large_startup_motion():
    validator = ViconPoseValidator(
        sample_count=2, timeout_s=0.25, max_spread_m=0.05
    )
    validator.add((0.0, 0.0, 0.5), (0.0, 0.0, 0.0, 1.0), 0.0)
    validator.add((0.2, 0.0, 0.5), (0.0, 0.0, 0.0, 1.0), 0.02)
    assert validator.is_ready(0.02) is False
