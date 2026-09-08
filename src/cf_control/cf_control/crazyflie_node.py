import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist


class CrazyflieNode(Node):

    def __init__(self):
        super().__init__('crazyflie_node')

        self.current_position = (
            0.0,
            0.0,
            0.0
        )

        self.position_pub = self.create_publisher(
            Point,
            '/cf1/position',
            10
        )

        self.target_sub = self.create_subscription(
            Point,
            '/cf1/target',
            self.target_callback,
            10
        )

    def target_callback(self, msg):

        self.get_logger().info(
            f'Target: {msg.x}, {msg.y}, {msg.z}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = CrazyflieNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()