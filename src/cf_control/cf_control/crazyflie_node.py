import rclpy
import cflib.crtp
import math
import time
from threading import Lock
from rclpy.executors import ExternalShutdownException
from cf_control.telemetry import TelemetryReader
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from rclpy.node import Node
from cf_control.lighthouse import LighthouseManager
from cf_control.flight_controller import FlightController
from geometry_msgs.msg import (Point, Twist)
from cf_control.config import (
    DRONES,
    CACHE_DIR,
    TELEMETRY_PERIOD_MS,
    POSITION_KP,
    MAX_VELOCITY,
    CONTROL_RATE_HZ,
    POSITION_TIMEOUT_S
)

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

        self.lighthouse = LighthouseManager(self.scf)

        self.current_position = None
        self.position_received_at = None
        self.position_lock = Lock()

        self.flight_controller = FlightController(
                    kp=POSITION_KP,
                    max_velocity=MAX_VELOCITY
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

        self.telemetry = TelemetryReader(
            self.scf,
            callback = self._on_position,
            period_ms=  TELEMETRY_PERIOD_MS
        )

        self.velocity_debug_pub = self.create_publisher(
            Twist,
            f'/{self.drone_id}/debug/velocity',
            10
        )

        self.control_timer = self.create_timer(
            1.0 / CONTROL_RATE_HZ,
            self.control_callback,

        )

    def target_callback(self, msg):
        target = (msg.x, msg.y, msg.z)

        if not all(math.isfinite(value) for value in target):
            self.get_logger().warning(
                'Ignoring target: coordinates must be finite'
            )
            return

        self.flight_controller.set_target(
            msg.x,
            msg.y,
            msg.z
        )

        self.get_logger().info(
            f'Target stored: {msg.x}, {msg.y}, {msg.z}'
        )

    def connect(self):
        self.get_logger().info(
            f'Connecting to {self.drone_id}: {self.uri}'
        )
        self.scf.open_link()

        self.get_logger().info('Requesting estimator reset')
        self.lighthouse.reset_estimator()

        self.get_logger().info(
            'Checking Lighthouse input and estimator convergence'
        )
        self.lighthouse.verify(timeout_s=10.0)

        self.get_logger().info(
            'Lighthouse input and variance stability checks passed'
        )
        self.telemetry.start()

    def _on_position(self, data, received_at):
        position = (
            float(data['stateEstimate.x']),
            float(data['stateEstimate.y']),
            float(data['stateEstimate.z'])
        )

        valid = all(math.isfinite(value) for value in position)

        with self.position_lock:
            self.current_position = position if valid else None
            self.position_received_at = received_at if valid else None

        if not valid or not self.context.ok():
            return

        msg = Point()
        msg.x, msg.y, msg.z = position
        self.position_pub.publish(msg)

    def control_callback(self):
        with self.position_lock:
            position = self.current_position
            received_at = self.position_received_at

        msg = Twist()  # All components initially zero.

        if (
            self.flight_controller.target is not None
            and position is not None
            and received_at is not None
            and time.monotonic() - received_at <= POSITION_TIMEOUT_S
        ):
            vx, vy, vz = self.flight_controller.update(position)

            msg.linear.x = vx
            msg.linear.y = vy
            msg.linear.z = vz

        self.velocity_debug_pub.publish(msg)

    def disconnect(self):
        self.control_timer.cancel()

        try:
            self.telemetry.stop()
        finally:
            self.scf.close_link()

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