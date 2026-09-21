"""ROS 2 node for prototyping wand input from the keyboard."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String

from cf_control.config import (
    VALID_WAND_GESTURES,
    WAND_CONNECTED_TOPIC,
    WAND_ENABLED_TOPIC,
    WAND_FRAME_ID,
    WAND_GESTURE_TOPIC,
    WAND_POSE_TOPIC,
    WAND_SIMULATION_STEP_M,
)


def normalize_gesture(value):
    """Return a normalized gesture name, or None when it is unsupported."""
    gesture = value.strip().lower().replace(' ', '_')
    if gesture in VALID_WAND_GESTURES:
        return gesture
    return None


def move_simulated_position(position, gesture, step=WAND_SIMULATION_STEP_M):
    """Return a new simulated position after applying a movement gesture."""
    x, y, z = position
    offsets = {
        'move_forward': (step, 0.0, 0.0),
        'move_backward': (-step, 0.0, 0.0),
        'move_left': (0.0, step, 0.0),
        'move_right': (0.0, -step, 0.0),
    }
    dx, dy, dz = offsets.get(gesture, (0.0, 0.0, 0.0))
    return (x + dx, y + dy, z + dz)


class WandNode(Node):
    """Publish validated, simulated wand gestures for early integration."""

    def __init__(self):
        super().__init__('wand_node')
        self.enabled = False
        self.position = (0.0, 0.0, 0.5)
        self.gesture_pub = self.create_publisher(
            String,
            WAND_GESTURE_TOPIC,
            10,
        )
        self.pose_pub = self.create_publisher(PoseStamped, WAND_POSE_TOPIC, 10)
        self.enabled_pub = self.create_publisher(Bool, WAND_ENABLED_TOPIC, 10)
        self.connected_pub = self.create_publisher(
            Bool,
            WAND_CONNECTED_TOPIC,
            10,
        )
        self.get_logger().info(
            f'Ready to publish simulated gestures on {WAND_GESTURE_TOPIC}'
        )
        self.publish_state()

    def publish_state(self):
        """Publish the simulator's current pose and device state."""
        connected = Bool()
        connected.data = True
        self.connected_pub.publish(connected)

        enabled = Bool()
        enabled.data = self.enabled
        self.enabled_pub.publish(enabled)

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = WAND_FRAME_ID
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (
            self.position
        )
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)

    def publish_gesture(self, value):
        """Validate and publish one gesture; return whether it was accepted."""
        gesture = normalize_gesture(value)
        if gesture is None:
            self.get_logger().warning(f'Unsupported gesture: {value!r}')
            return False

        if gesture == 'enable':
            self.enabled = True
        elif gesture == 'disable':
            self.enabled = False
        elif gesture.startswith('move_'):
            if not self.enabled:
                self.get_logger().warning(
                    "Wand is disabled; enter 'enable' before moving"
                )
                self.publish_state()
                return False
            self.position = move_simulated_position(self.position, gesture)

        message = String()
        message.data = gesture
        self.gesture_pub.publish(message)
        self.publish_state()
        self.get_logger().info(f'Published gesture: {gesture}')
        return True


def main(args=None):
    """Run an interactive keyboard prototype of the wand input node."""
    rclpy.init(args=args)
    node = WandNode()
    choices = ', '.join(sorted(VALID_WAND_GESTURES))

    try:
        print(f'Available gestures: {choices}')
        print("Type a gesture, or 'quit' to stop.")
        while rclpy.ok():
            value = input('wand> ')
            if value.strip().lower() in {'quit', 'exit'}:
                break
            node.publish_gesture(value)
            rclpy.spin_once(node, timeout_sec=0.0)
    except (EOFError, KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
