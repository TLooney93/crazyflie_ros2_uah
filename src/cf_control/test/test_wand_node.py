"""Tests for wand gesture validation."""

from cf_control.wand_node import move_simulated_position, normalize_gesture


def test_normalize_gesture_accepts_supported_input():
    assert normalize_gesture(' Move Left ') == 'move_left'
    assert normalize_gesture('TAKEOFF') == 'takeoff'


def test_normalize_gesture_rejects_unsupported_input():
    assert normalize_gesture('flip') is None
    assert normalize_gesture('') is None


def test_move_simulated_position_updates_expected_axis():
    position = (1.0, 2.0, 0.5)

    assert move_simulated_position(
        position,
        'move_forward',
    ) == (1.1, 2.0, 0.5)
    assert move_simulated_position(position, 'move_left') == (1.0, 2.1, 0.5)


def test_non_movement_gesture_does_not_change_position():
    position = (1.0, 2.0, 0.5)

    assert move_simulated_position(position, 'hold') == position
