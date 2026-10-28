"""Read physical wand telemetry and publish it through ROS 2."""

import math

import cflib.crtp
import rclpy
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from cf_control.config import (
    CACHE_DIR,
    DRONES,
    TELEMETRY_PERIOD_MS,
    WAND_FRAME_ID,
    WAND_POSE_TOPIC,
)
from cf_control.telemetry import TelemetryReader


def euler_degrees_to_quaternion(roll, pitch, yaw):
    """Convert Euler angles in degrees to a quaternion."""
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)

    cy = math.cos(yaw / 2.0)
    sy = math.sin(yaw / 2.0)
    cp = math.cos(pitch / 2.0)
    sp = math.sin(pitch / 2.0)
    cr = math.cos(roll / 2.0)
    sr = math.sin(roll / 2.0)

    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy

    return x, y, z, w


class WandDriverNode(Node):
    """ROS 2 node for reading wand telemetry and publishing it."""

    def __init__(self):
        super().__init__('wand_driver_node')
        self.scf = None
        self.telemetry = None
        self.declare_parameter('wand_id', 'cf1')
        self.wand_id = self.get_parameter('wand_id').value
        if self.wand_id not in DRONES:
            raise ValueError(f'Unknown wand ID: {self.wand_id}')
        self.uri = DRONES[self.wand_id]
        self.get_logger().info('Wand driver node started')
        self.pose_pub = self.create_publisher(
            PoseStamped,
            WAND_POSE_TOPIC,
            10,
        )
        
    def _on_telemetry(self, data, received_at):
        """Convert one telemetry sample into a ROS pose."""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = WAND_FRAME_ID

        pose_msg.pose.position.x = float(data['stateEstimate.x'])
        pose_msg.pose.position.y = float(data['stateEstimate.y'])
        pose_msg.pose.position.z = float(data['stateEstimate.z'])

        roll = float(data['stabilizer.roll'])
        pitch = float(data['stabilizer.pitch'])
        yaw = float(data['stabilizer.yaw'])
        qx, qy, qz, qw = euler_degrees_to_quaternion(
            roll,
            pitch,
            yaw,
        )

        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        self.pose_pub.publish(pose_msg)

    def connect(self):
        """Open the wand radio link and begin reading telemetry."""
        self.scf = SyncCrazyflie(
            self.uri,
            cf=Crazyflie(rw_cache=str(CACHE_DIR)),
        )
        self.scf.open_link()

        self.telemetry = TelemetryReader(
            self.scf,
            callback=self._on_telemetry,
            period_ms=TELEMETRY_PERIOD_MS,
        )
        self.telemetry.start()
        self.get_logger().info(f'Connected to wand: {self.wand_id}')

    def disconnect(self):
        """Stop telemetry and close the wand radio link."""
        try:
            if self.telemetry is not None:
                self.telemetry.stop()
        finally:
            if self.scf is not None:
                self.scf.close_link()
        self.get_logger().info('Wand disconnected')


def main(args=None):
    """Run the physical wand driver node."""
    rclpy.init(args=args)
    node = None

    try:
        cflib.crtp.init_drivers()
        node = WandDriverNode()
        node.connect()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.disconnect()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()