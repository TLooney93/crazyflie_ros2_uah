"""Tests for project-wide configuration invariants."""

from cf_control.config import validate_config


def test_runtime_configuration_is_internally_consistent():
    """The checked-in runtime defaults must satisfy all safety invariants."""
    validate_config()
