"""Recognize vertical wand gestures from tracked poses and button events."""

import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String

from cf_control.vertical_gesture import VerticalGesture


class WandGestureNode(Node):
    """Publish detections separately from flight commands by default."""

    def __init__(self):
        super().__init__('wand_gesture_node')
        for name, value in (
            ('pose_topic', '/wand/pose'),
            ('output_topic', '/wand/detected_gesture'),
            ('distance_m', 0.20), ('pose_timeout_s', 0.25),
            ('confirmation_samples', 3), ('frame_id', 'world'),
        ):
            self.declare_parameter(name, value)
        self.detector = VerticalGesture(
            self.get_parameter('distance_m').value,
            self.get_parameter('pose_timeout_s').value,
            self.get_parameter('confirmation_samples').value,
        )
        self.frame = self.get_parameter('frame_id').value
        self.last_stamp = None
        self.publisher = self.create_publisher(
            String, self.get_parameter('output_topic').value, 10)
        self.buttons = self.create_subscription(
            String, '/wand/gesture', self.on_control, 10)
        self.poses = self.create_subscription(
            PoseStamped, self.get_parameter('pose_topic').value,
            self.on_pose, qos_profile_sensor_data)
        self.watchdog = self.create_timer(
            0.05, lambda: self.detector.expire(time.monotonic()))
        self.get_logger().info('Vertical gestures ready; disabled until IO1 press')

    def on_control(self, msg):
        self.detector.control(msg.data)

    def on_pose(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if (msg.header.frame_id != self.frame or stamp <= 0
                or age > self.detector.timeout or age < -0.05
                or (self.last_stamp is not None and stamp <= self.last_stamp)):
            self.detector.control('disable')
            return
        self.last_stamp = stamp
        p = msg.pose.position
        gesture = self.detector.update((p.x, p.y, p.z), time.monotonic())
        if gesture:
            result = String()
            result.data = gesture
            self.publisher.publish(result)
            self.get_logger().info(f'Detected {gesture}; release IO1 to rearm')


def main(args=None):
    rclpy.init(args=args)
    node = WandGestureNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
