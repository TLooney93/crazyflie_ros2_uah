import rclpy
import cflib.crtp
from rclpy.executors import ExternalShutdownException
from cf_control.config import (
    CACHE_DIR,
    CONTROL_RATE_HZ,
    DRONES,
    MAX_VELOCITY,
    POSITION_KP,
    TELEMETRY_PERIOD_MS,
)
from cf_control.crazyflie_interface import CrazyflieInterface
from cf_control.flight_controller import FlightController
from cf_control.telemetry import TelemetryReader
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from rclpy.node import Node

from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist


class CrazyflieNode(Node):

    def __init__(self):
        super().__init__('crazyflie_node')

        self.declare_parameter('drone_id', 'cf1')
        self.drone_id = self.get_parameter('drone_id').value

        if self.drone_id not in DRONES:
            raise ValueError(f"Uknown drone name: {self.drone_id}")

        self.uri = DRONES[self.drone_id]

        self.scf = SyncCrazyflie(
            self.uri,
            cf=Crazyflie(rw_cache=str(CACHE_DIR))
        )

        self.current_position = None
        self.connected = False
        self.interface = CrazyflieInterface(self.scf)
        self.controller = FlightController(
            kp=POSITION_KP,
            max_velocity=MAX_VELOCITY,
        )

        self.position_pub = self.create_publisher(
            Point,
            f'/{self.drone_id}/position',
            10
        )

        self.target_sub = self.create_subscription(
            Point,
            f'/{self.drone_id}/target',
            self.target_callback,
            10
        )

        self.control_timer = self.create_timer(
            1.0 / CONTROL_RATE_HZ,
            self.control_step,
        )

        self.telemetry = TelemetryReader(
            self.scf,
            callback = self._on_position,
            period_ms=  TELEMETRY_PERIOD_MS
        )

    def target_callback(self, msg):

        self.controller.set_target(msg.x, msg.y, msg.z)

        self.get_logger().info(
            f'Target: {msg.x}, {msg.y}, {msg.z}'
        )

    def control_step(self):
        """Send one bounded velocity command toward the active target."""
        if self.current_position is None or self.controller.target is None:
            return

        vx, vy, vz = self.controller.update(self.current_position)
        self.interface.send_velocity(vx, vy, vz)

    def connect(self):
        self.get_logger().info(f'Connecting to {self.drone_id}: {self.uri}')
        self.scf.open_link()
        self.connected = True
        self.telemetry.start()
        self.get_logger().info('Radio connected; waiting for position telemetry')

    def _on_position(self, data):
        position = (
            float(data['stateEstimate.x']),
            float(data['stateEstimate.y']),
            float(data['stateEstimate.z'])
        )
        self.current_position = position

        if not self.context.ok():
            return

        msg = Point()
        msg.x, msg.y, msg.z = position
        self.position_pub.publish(msg)

    def disconnect(self):
        if not self.connected:
            return

        try:
            self.telemetry.stop()
        finally:
            try:
                self.interface.stop()
            finally:
                self.scf.close_link()
                self.connected = False

def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        cflib.crtp.init_drivers()
        node = CrazyflieNode()
        node.connect()
        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:
        try:
            if node is not None:
                try:
                    node.disconnect()
                finally:
                    node.destroy_node()
        finally:
            rclpy.try_shutdown()
