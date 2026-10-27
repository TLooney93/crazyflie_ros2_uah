"""Read a physical Crazyflie wand and publish its tracked pose."""

from cf_control.config import (
    CACHE_DIR,
    DRONES,
    TELEMETRY_PERIOD_MS,
    WAND_CONNECTED_TOPIC,
    WAND_FRAME_ID,
    WAND_POSE_TOPIC,
)
from cf_control.telemetry import TelemetryReader
from cf_control.wand_control import euler_degrees_to_quaternion
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
import cflib.crtp
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool


class WandDriverNode(Node):
    """Publish position and attitude telemetry from a physical wand."""

    def __init__(self):
        super().__init__('wand_driver_node')
        self.declare_parameter('wand_id', 'cf2')
        self.declare_parameter('wand_uri', '')
        self.wand_id = self.get_parameter('wand_id').value
        uri_override = self.get_parameter('wand_uri').value
        if self.wand_id not in DRONES:
            raise ValueError(f'Unknown wand ID: {self.wand_id}')
        self.uri = uri_override or DRONES[self.wand_id]
        self.scf = None
        self.telemetry = None
        self.connected = False
        self.pose_pub = self.create_publisher(PoseStamped, WAND_POSE_TOPIC, 10)
        self.connected_pub = self.create_publisher(
            Bool,
            WAND_CONNECTED_TOPIC,
            10,
        )
        self.connection_timer = self.create_timer(
            1.0,
            self._publish_connection_state,
        )

    def _publish_connection_state(self):
        """Publish the current radio-connection state."""
        message = Bool()
        message.data = self.connected
        self.connected_pub.publish(message)

    def _on_telemetry(self, data, received_at):
        """Convert one telemetry sample into a ROS pose."""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = WAND_FRAME_ID
        pose.pose.position.x = float(data['stateEstimate.x'])
        pose.pose.position.y = float(data['stateEstimate.y'])
        pose.pose.position.z = float(data['stateEstimate.z'])
        quaternion = euler_degrees_to_quaternion(
            float(data['stabilizer.roll']),
            float(data['stabilizer.pitch']),
            float(data['stabilizer.yaw']),
        )
        (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ) = quaternion
        self.pose_pub.publish(pose)

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
            include_attitude=True,
        )
        self.telemetry.start()
        self.connected = True
        self._publish_connection_state()
        self.get_logger().info(
            f'Connected to physical wand {self.wand_id}: {self.uri}'
        )

    def disconnect(self):
        """Stop telemetry and close the wand radio link."""
        self.connected = False
        self._publish_connection_state()
        try:
            if self.telemetry is not None:
                self.telemetry.stop()
        finally:
            if self.scf is not None:
                self.scf.close_link()


def main(args=None):
    """Run the physical wand driver."""
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
