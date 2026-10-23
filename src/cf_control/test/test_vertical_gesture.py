"""Simulated pose sequences for the vertical gesture detector."""

import math
import pytest

from cf_control.vertical_gesture import VerticalGesture


@pytest.mark.parametrize('height, expected', [(1.3, 'takeoff'), (0.7, 'land')])
def test_vertical_motion_once_per_hold(height, expected):
    detector = VerticalGesture()
    detector.control('enable')
    assert detector.update((0, 0, 1), 0) is None
    assert detector.update((0, 0, height), 0.05) is None
    assert detector.update((0, 0, height), 0.10) is None
    assert detector.update((0, 0, height), 0.15) == expected
    detector.control('enable')  # Duplicate button event must not rearm.
    assert detector.update((0, 0, height), 0.20) is None
    detector.control('disable')
    detector.control('enable')
    assert detector.update((0, 0, height), 0.25) is None
    for t in (0.30, 0.35):
        assert detector.update((0, 0, height + 0.3), t) is None
    assert detector.update((0, 0, height + 0.3), 0.40) == 'takeoff'


@pytest.mark.parametrize('control', ['disable', 'emergency_stop'])
def test_release_or_stop_blocks_motion(control):
    detector = VerticalGesture()
    detector.control('enable')
    detector.update((0, 0, 1), 0)
    detector.control(control)
    for t in (0.05, 0.10, 0.15):
        assert detector.update((0, 0, 2), t) is None


def test_disabled_noise_horizontal_and_single_spike_do_not_fire():
    detector = VerticalGesture()
    assert detector.update((0, 0, 1), 0) is None
    detector.control('enable')
    heights = [1, 1.02, 1.01, 1.3, 1.0, 1.01]
    for i, height in enumerate(heights):
        assert detector.update((i, i, height), i * 0.05) is None


def test_stale_tracking_requires_new_enable():
    detector = VerticalGesture()
    detector.control('enable')
    detector.update((0, 0, 1), 0)
    assert detector.update((0, 0, 2), 1) is None
    assert not detector.enabled


def test_invalid_pose_disables():
    detector = VerticalGesture()
    detector.control('enable')
    detector.update((0, 0, math.nan), 0)
    assert not detector.enabled


def test_emergency_stop_bypasses_pending_service():
    from unittest.mock import Mock
    from cf_control.wand_coordinator import WandCoordinator
    node = Mock()
    old = Mock()
    node.pending_service = old
    client = Mock()
    client.service_is_ready.return_value = True
    WandCoordinator._call(node, client, 'emergency stop', priority=True)
    client.call_async.assert_called_once()
    assert node.pending_service is client.call_async.return_value


def test_ros_pose_callback_rejects_stale_and_accepts_vertical_motion():
    from unittest.mock import Mock
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from cf_control.wand_gesture_node import WandGestureNode
    rclpy.init()
    node = WandGestureNode()
    try:
        node.publisher = Mock()
        node.detector.control('enable')
        for height in (1.0, 1.3, 1.3, 1.3):
            pose = PoseStamped()
            pose.header.frame_id = 'world'
            pose.header.stamp = node.get_clock().now().to_msg()
            pose.pose.position.z = height
            node.on_pose(pose)
        assert node.publisher.publish.call_count == 1
        assert node.publisher.publish.call_args.args[0].data == 'takeoff'
        node.detector.control('disable')
        node.detector.control('enable')
        pose.header.stamp.sec -= 5
        node.on_pose(pose)
        assert not node.detector.enabled
    finally:
        node.destroy_node()
        rclpy.shutdown()
