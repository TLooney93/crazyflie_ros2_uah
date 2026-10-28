"""Recognize mission gestures from a Vicon-tracked wand pose."""

import time

from cf_control.config import (
    WAND_GESTURE_BASELINE_ALPHA,
    WAND_GESTURE_COOLDOWN_S,
    WAND_GESTURE_HORIZONTAL_TRAVEL_M,
    WAND_GESTURE_MAX_DURATION_S,
    WAND_GESTURE_RETURN_TOLERANCE_M,
    WAND_GESTURE_TOPIC,
    WAND_GESTURE_VERTICAL_TRAVEL_M,
    WAND_VICON_POSE_TOPIC,
)
from cf_control.wand_control import PositionGestureRecognizer
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class WandGestureNode(Node):
    """Publish one named event after a complete tracked wand gesture."""

    def __init__(self):
        super().__init__('wand_gesture_node')
        self.declare_parameter('pose_topic', WAND_VICON_POSE_TOPIC)
        self.declare_parameter(
            'vertical_travel_m',
            WAND_GESTURE_VERTICAL_TRAVEL_M,
        )
        self.declare_parameter(
            'horizontal_travel_m',
            WAND_GESTURE_HORIZONTAL_TRAVEL_M,
        )
        self.declare_parameter(
            'return_tolerance_m',
            WAND_GESTURE_RETURN_TOLERANCE_M,
        )
        self.declare_parameter(
            'max_duration_s',
            WAND_GESTURE_MAX_DURATION_S,
        )
        self.declare_parameter('cooldown_s', WAND_GESTURE_COOLDOWN_S)
        self.declare_parameter(
            'baseline_alpha',
            WAND_GESTURE_BASELINE_ALPHA,
        )
        pose_topic = self.get_parameter('pose_topic').value
        if not pose_topic:
            raise ValueError('pose_topic must not be empty')

        self.recognizer = PositionGestureRecognizer(
            vertical_travel_m=self.get_parameter(
                'vertical_travel_m'
            ).value,
            horizontal_travel_m=self.get_parameter(
                'horizontal_travel_m'
            ).value,
            return_tolerance_m=self.get_parameter(
                'return_tolerance_m'
            ).value,
            max_duration_s=self.get_parameter('max_duration_s').value,
            cooldown_s=self.get_parameter('cooldown_s').value,
            baseline_alpha=self.get_parameter('baseline_alpha').value,
        )
        self.gesture_pub = self.create_publisher(
            String,
            WAND_GESTURE_TOPIC,
            10,
        )
        self.pose_sub = self.create_subscription(
            PoseStamped,
            pose_topic,
            self._on_pose,
            10,
        )
        self.get_logger().info(
            f'Recognizing Vicon wand gestures from {pose_topic}'
        )

    def _on_pose(self, message):
        position = (
            message.pose.position.x,
            message.pose.position.y,
            message.pose.position.z,
        )
        try:
            gesture = self.recognizer.update(position, time.monotonic())
        except ValueError as error:
            self.get_logger().warning(f'Ignoring invalid wand pose: {error}')
            return
        if gesture is None:
            return

        output = String()
        output.data = gesture
        self.gesture_pub.publish(output)
        self.get_logger().info(f'Recognized wand gesture: {gesture}')


def main(args=None):
    """Run the Vicon position gesture recognizer."""
    rclpy.init(args=args)
    node = None
    try:
        node = WandGestureNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
