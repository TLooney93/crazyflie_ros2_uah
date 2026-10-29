"""Read a physical Crazyflie wand and publish its tracked pose."""

from cf_control.config import (
    CACHE_DIR,
    DRONES,
    TELEMETRY_PERIOD_MS,
    WAND_CONNECTED_TOPIC,
    WAND_ENABLED_TOPIC,
    WAND_FRAME_ID,
    WAND_GESTURE_TOPIC,
    WAND_POSE_TOPIC,
)
from cf_control.telemetry import TelemetryReader
from cf_control.wand_control import ButtonEdges, euler_degrees_to_quaternion
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
import cflib.crtp
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String


class WandDriverNode(Node):
    """Publish position and attitude telemetry from a physical wand."""

    BUTTONS = (
        ('wand.stop', 'emergency_stop'),
        ('wand.disable', 'disable'),
        ('wand.enable', 'enable'),
        ('wand.takeoff', 'takeoff'),
        ('wand.land', 'land'),
    )

    def __init__(self, node_name='wand_driver_node', parameter_overrides=None):
        super().__init__(node_name, parameter_overrides=parameter_overrides)
        self.declare_parameter('wand_id', 'cf2')
        self.declare_parameter('wand_uri', '')
        self.declare_parameter('buttons_enabled', True)
        self.wand_id = self.get_parameter('wand_id').value
        uri_override = self.get_parameter('wand_uri').value
        if self.wand_id not in DRONES:
            raise ValueError(f'Unknown wand ID: {self.wand_id}')
        self.uri = uri_override or DRONES[self.wand_id]
        self.buttons_enabled = bool(
            self.get_parameter('buttons_enabled').value
        )
        self.scf = None
        self.telemetry = None
        self.button_log = None
        self.button_edges = ButtonEdges()
        self.enabled = False
        self.connected = False
        self.pose_pub = self.create_publisher(PoseStamped, WAND_POSE_TOPIC, 10)
        self.connected_pub = self.create_publisher(
            Bool,
            WAND_CONNECTED_TOPIC,
            10,
        )
        self.enabled_pub = self.create_publisher(Bool, WAND_ENABLED_TOPIC, 10)
        self.gesture_pub = self.create_publisher(
            String,
            WAND_GESTURE_TOPIC,
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
        enabled = Bool()
        enabled.data = self.enabled
        self.enabled_pub.publish(enabled)

    def _on_buttons(self, timestamp, data, log_config):
        """Publish one normalized event for each firmware button pulse."""
        for variable, gesture in self.BUTTONS:
            if not self.button_edges.pressed(variable, data.get(variable, 0)):
                continue
            if gesture == 'enable':
                self.enabled = True
            elif gesture in ('disable', 'emergency_stop'):
                self.enabled = False

            message = String()
            message.data = gesture
            self.gesture_pub.publish(message)
            self._publish_connection_state()
            log = (
                self.get_logger().error
                if gesture == 'emergency_stop'
                else self.get_logger().info
            )
            log(f'Wand button event: {gesture}')

    def _on_button_log_error(self, log_config, message):
        """Report failures in the custom firmware button log block."""
        self.get_logger().error(f'Wand button log error: {message}')

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
        if self.buttons_enabled:
            self.button_log = LogConfig(name='WandButtons', period_in_ms=50)
            for variable, _gesture in self.BUTTONS:
                self.button_log.add_variable(variable, 'uint8_t')
            self.scf.cf.log.add_config(self.button_log)
            if not self.button_log.valid:
                raise RuntimeError(
                    'Wand firmware does not expose the wand.* button logs'
                )
            self.button_log.data_received_cb.add_callback(self._on_buttons)
            self.button_log.error_cb.add_callback(
                self._on_button_log_error
            )
            self.button_log.start()
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
            if self.button_log is not None:
                self.button_log.stop()
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
