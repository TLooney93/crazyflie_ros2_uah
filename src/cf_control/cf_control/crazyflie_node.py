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
from cf_control.crazyflie_interface import CrazyflieInterface
from geometry_msgs.msg import (Point, Twist)
from cf_control.flight_state import FlightState
from std_srvs.srv import Trigger
from cf_control.config import (
    DRONES,
    CACHE_DIR,
    TELEMETRY_PERIOD_MS,
    POSITION_KP,
    MAX_VELOCITY,
    CONTROL_RATE_HZ,
    POSITION_TIMEOUT_S,
    DEFAULT_HEIGHT,
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

        self.command_interface = CrazyflieInterface(self.scf)

        self.declare_parameter('enable_actuation', False)
        self.enable_actuation = bool(
            self.get_parameter('enable_actuation').value
        )

        self.declare_parameter('takeoff_height', DEFAULT_HEIGHT)
        self.takeoff_height = float(
            self.get_parameter('takeoff_height').value
        )

        self.declare_parameter('takeoff_duration_s', 2.0)
        self.takeoff_duration_s = float(
            self.get_parameter('takeoff_duration_s').value
        )

        self.lighthouse = LighthouseManager(self.scf)

        self.declare_parameter('land_duration_s', 2.0)
        self.land_duration_s = float(
            self.get_parameter('land_duration_s').value
        )
        self.landing_timer = None
        self.current_position = None
        self.position_received_at = None
        self.position_lock = Lock()

        self.flight_controller = FlightController(
                    kp=POSITION_KP,
                    max_velocity=MAX_VELOCITY
                )

        self.flight_state = FlightState.DISARMED

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

        self.clear_target_service = self.create_service(
            Trigger,
            f'/{self.drone_id}/clear_target',
            self.clear_target_callback,
        )

        self.takeoff_service = self.create_service(
            Trigger,
            f'/{self.drone_id}/takeoff',
            self.takeoff_callback,
        )

        self.land_service = self.create_service(
            Trigger,
            f'/{self.drone_id}/land',
            self.land_callback,
        )

        self.emergency_stop_service = self.create_service(
            Trigger,
            f'/{self.drone_id}/emergency_stop',
            self.emergency_stop_callback,
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

    def clear_target_callback(self, request, response):
        """Clear the target used by the dry-run controller."""
        self.flight_controller.clear_target()

        self.get_logger().info('Target cleared')

        response.success = True
        response.message = (
            'Target cleared; diagnostic velocity will be zero '
            'on the next control tick.'
        )
        return response

    def takeoff_callback(self, request, response):
        """Request a guarded high-level takeoff."""
        if not self.enable_actuation:
            response.success = False
            response.message = 'Actuation is disabled; no takeoff sent.'
            return response

        if self.flight_state is not FlightState.READY:
            response.success = False
            response.message = (
                f'Takeoff rejected in state {self.flight_state.name}'
            )
            return response

        self.command_interface.arm()
        self.command_interface.takeoff(
            self.takeoff_height,
            self.takeoff_duration_s,
        )
        self.flight_state = FlightState.FLYING

        response.success = True
        response.message = (
            f'Takeoff requested to {self.takeoff_height:.2f} metres.'
        )
        return response

    def land_callback(self, request, response):
        """Request a guarded high-level landing."""
        if not self.enable_actuation:
            response.success = False
            response.message = 'Actuation is disabled; no landing sent.'
            return response

        if self.flight_state is not FlightState.FLYING:
            response.success = False
            response.message = (
                f'Landing rejected in state {self.flight_state.name}'
            )
            return response

        self.command_interface.land(
            height_m=0.0,
            duration_s=self.land_duration_s,
        )
        self.landing_timer = self.create_timer(
            self.land_duration_s,
            self.finish_landing_callback,
        )

        self.flight_state = FlightState.LANDING

        response.success = True
        response.message = (
            f'Landing requested to 0.00 metres over '
            f'{self.land_duration_s:.2f} seconds.'
        )
        return response

    def finish_landing_callback(self):
        """Stop and disarm after the landing duration."""
        if self.landing_timer is not None:
            self.landing_timer.cancel()
            self.landing_timer = None
        
        self.command_interface.stop_high_level()
        self.command_interface.disarm()
        self.flight_state = FlightState.DISARMED
        self.get_logger().info('Landing complete; motors disarmed')

    def emergency_stop_callback(self, request, response):
        """Immediately stop commands and disarm the drone."""
        if not self.enable_actuation:
            response.success = False
            response.message = 'Actuation is disabled; no emergency stop sent.'
            return response

        if self.landing_timer is not None:
            self.landing_timer.cancel()
            self.landing_timer = None

        self.flight_controller.clear_target()
        self.command_interface.stop_high_level()
        self.command_interface.disarm()
        self.flight_state = FlightState.DISARMED

        self.get_logger().warning('EMERGENCY STOP: Motors disarmed')
        response.success = True
        response.message = 'Emergency stop sent; motors disarmed.'
        return response

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
        self.flight_state = FlightState.READY

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

if __name__ == '__main__':
    main()