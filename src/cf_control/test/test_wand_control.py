"""Tests for wand gesture and orientation helpers."""

import math
from types import SimpleNamespace

from cf_control.crazyflie_node import CrazyflieNode
from cf_control.flight_controller import FlightController
from cf_control.flight_state import FlightState
from cf_control.wand_control import (
    ButtonEdges,
    euler_degrees_to_quaternion,
    gesture_action,
    move_simulated_position,
    normalize_gesture,
    PositionGestureRecognizer,
    tilt_to_unit_offset,
)


class _RecordingLogger:
    """Collect messages emitted by unbound controller callback tests."""

    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(message)

    def warning(self, message):
        self.messages.append(message)


def test_normalize_gesture_accepts_only_supported_input():
    assert normalize_gesture(' Move Left ') == 'move_left'
    assert normalize_gesture('TAKEOFF') == 'takeoff'
    assert normalize_gesture('flip') is None


def test_physical_gestures_map_to_guarded_actions():
    assert gesture_action('up_down') == 'takeoff'
    assert gesture_action('side_to_side') == 'start_mission'
    assert gesture_action('down_up') == 'land'
    assert gesture_action('emergency_stop') == 'emergency_stop'


def test_disable_during_wand_takeoff_preserves_target_and_phase():
    """Releasing capture must not cancel an accepted takeoff command."""
    logger = _RecordingLogger()
    controller = FlightController(kp=0.5, max_velocity=0.3)
    controller.set_target(1.0, 2.0, 0.4)
    node = SimpleNamespace(
        flight_state=FlightState.FLYING,
        test_phase='WAND_TAKEOFF',
        wand_active=True,
        wand_reference_orientation=(0.0, 0.0, 0.0, 1.0),
        wand_last_move_at=1.0,
        flight_controller=controller,
        get_logger=lambda: logger,
    )

    CrazyflieNode.wand_gesture_callback(
        node,
        SimpleNamespace(data='disable'),
    )

    assert not node.wand_active
    assert node.test_phase == 'WAND_TAKEOFF'
    assert controller.target == (1.0, 2.0, 0.4)


def test_enable_during_mission_preserves_target_and_phase():
    """Opening capture must not interrupt an active saved mission."""
    logger = _RecordingLogger()
    controller = FlightController(kp=0.5, max_velocity=0.3)
    controller.set_target(0.2, -0.1, 0.5)
    orientation = (0.0, 0.0, 0.0, 1.0)
    node = SimpleNamespace(
        flight_state=FlightState.FLYING,
        test_phase='FIGURE_EIGHT',
        wand_active=False,
        wand_orientation=orientation,
        wand_reference_orientation=None,
        wand_last_move_at=1.0,
        flight_controller=controller,
        get_logger=lambda: logger,
    )

    CrazyflieNode.wand_gesture_callback(
        node,
        SimpleNamespace(data='enable'),
    )

    assert node.wand_active
    assert node.wand_reference_orientation == orientation
    assert node.test_phase == 'FIGURE_EIGHT'
    assert controller.target == (0.2, -0.1, 0.5)


def test_button_edges_publish_only_once_per_firmware_pulse():
    edges = ButtonEdges()

    assert not edges.pressed('wand.enable', 0)
    assert edges.pressed('wand.enable', 1)
    assert not edges.pressed('wand.enable', 1)
    assert not edges.pressed('wand.enable', 0)
    assert edges.pressed('wand.enable', 1)


def test_gesture_reset_requires_a_new_baseline():
    recognizer = PositionGestureRecognizer()

    assert recognizer.update((0.0, 0.0, 1.0), 0.0) is None
    assert recognizer.update((0.0, 0.0, 1.20), 0.2) is None
    recognizer.reset()
    assert recognizer.update((0.0, 0.0, 1.0), 0.3) is None


def test_up_down_gesture_requires_return_to_baseline():
    recognizer = PositionGestureRecognizer(cooldown_s=0.5)

    assert recognizer.update((0.0, 0.0, 1.0), 0.0) is None
    assert recognizer.update((0.0, 0.0, 1.20), 0.2) is None
    assert recognizer.update((0.0, 0.0, 1.10), 0.4) is None
    assert recognizer.update((0.0, 0.0, 1.02), 0.6) == 'up_down'
    assert recognizer.update((0.0, 0.0, 1.20), 0.7) is None


def test_down_up_gesture_is_recognized():
    recognizer = PositionGestureRecognizer()

    assert recognizer.update((0.0, 0.0, 1.0), 0.0) is None
    assert recognizer.update((0.0, 0.0, 0.80), 0.2) is None
    assert recognizer.update((0.0, 0.0, 0.98), 0.5) == 'down_up'


def test_side_to_side_uses_horizontal_motion_in_any_heading():
    recognizer = PositionGestureRecognizer()

    assert recognizer.update((1.0, 2.0, 0.5), 0.0) is None
    assert recognizer.update((0.8, 2.0, 0.5), 0.2) is None
    assert recognizer.update((1.2, 2.0, 0.5), 0.6) == 'side_to_side'


def test_incomplete_gesture_expires_without_an_event():
    recognizer = PositionGestureRecognizer(max_duration_s=1.0)

    assert recognizer.update((0.0, 0.0, 1.0), 0.0) is None
    assert recognizer.update((0.0, 0.0, 1.20), 0.2) is None
    assert recognizer.update((0.0, 0.0, 1.0), 1.3) is None


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
