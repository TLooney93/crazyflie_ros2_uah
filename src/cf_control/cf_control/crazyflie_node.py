import math
import time
from threading import Lock

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from cf_control.config import (
    ARM_SETTLE_TIME_S,
    CACHE_DIR,
    CONTROL_RATE_HZ,
    DRONES,
    ENABLE_EXTERNAL_TARGETS,
    FLIGHT_TEST_HEIGHT_M,
    LANDING_COMPLETE_TOLERANCE_M,
    LANDING_HEIGHT_M,
    LANDING_SPEED_M_S,
    MAX_FLIGHT_TIME_S,
    MAX_VELOCITY,
    POSITION_KP,
    POSITION_TIMEOUT_S,
    TELEMETRY_LOSS_DESCENT_S,
    TELEMETRY_PERIOD_MS,
)
from cf_control.crazyflie_interface import (
    CrazyflieInterface,
    SimulatedCrazyflieInterface,
)
from cf_control.flight_controller import FlightController
from cf_control.flight_state import FlightState
from cf_control.lighthouse import LighthouseManager
from cf_control.telemetry import TelemetryReader


class CrazyflieNode(Node):

    def __init__(self):
        super().__init__('crazyflie_node')

        self.declare_parameter('drone_id', 'cf1')
        self.declare_parameter('simulation', False)
        self.drone_id = self.get_parameter('drone_id').value
        self.simulation = self.get_parameter('simulation').value

        if self.drone_id not in DRONES:
            raise ValueError(f"Uknown drone name: {self.drone_id}")

        self.uri = DRONES[self.drone_id]

        self.scf = None
        self.lighthouse = None
        self.telemetry = None

        if not self.simulation:
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

        self.flight_state = FlightState.DISARMED
        self.home_position = None
        self.armed_at = None
        self.flight_started_at = None
        self.telemetry_lost_at = None

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

        if self.simulation:
            self.sim_velocity_pub = self.create_publisher(
                Twist,
                '/model/crazyflie/cmd_vel',
                10,
            )
            self.sim_odometry_sub = self.create_subscription(
                Odometry,
                '/model/crazyflie/odometry',
                self._on_sim_odometry,
                10,
            )
            self.interface = SimulatedCrazyflieInterface(
                self._publish_sim_velocity
            )
        else:
            self.interface = CrazyflieInterface(self.scf)
            self.telemetry = TelemetryReader(
                self.scf,
                callback=self._on_position,
                period_ms=TELEMETRY_PERIOD_MS
            )

        self.velocity_debug_pub = self.create_publisher(
            Twist,
            f'/{self.drone_id}/debug/velocity',
            10
        )

        self.state_pub = self.create_publisher(
            String,
            f'/{self.drone_id}/flight_state',
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

        self.arm_service = self.create_service(
            Trigger,
            f'/{self.drone_id}/arm',
            self.arm_callback,
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

        self._publish_state()

    def target_callback(self, msg):
        if not ENABLE_EXTERNAL_TARGETS:
            self.get_logger().warning(
                'Ignoring external target: disabled for the first flight test'
            )
            return

        if self.flight_state is not FlightState.FLYING:
            self.get_logger().warning(
                'Ignoring target: drone is not in FLYING state'
            )
            return

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

    def arm_callback(self, request, response):
        """Arm while grounded after confirming position telemetry is fresh."""
        if self.flight_state is not FlightState.DISARMED:
            return self._service_failure(
                response,
                f'Cannot arm from {self.flight_state.name}'
            )

        position = self._fresh_position()
        if position is None:
            return self._service_failure(
                response,
                'Cannot arm without fresh position telemetry'
            )

        self.home_position = position
        self.flight_controller.clear_target()
        self.interface.arm()
        self.armed_at = time.monotonic()
        self._set_state(FlightState.READY)
        response.success = True
        response.message = 'Armed and ready for vertical takeoff'
        return response

    def takeoff_callback(self, request, response):
        """Start the constrained vertical flight test."""
        if self.flight_state is not FlightState.READY:
            return self._service_failure(
                response,
                f'Cannot take off from {self.flight_state.name}'
            )

        if time.monotonic() - self.armed_at < ARM_SETTLE_TIME_S:
            return self._service_failure(
                response,
                'Arming is still settling; retry takeoff shortly'
            )

        if self._fresh_position() is None:
            return self._service_failure(
                response,
                'Cannot take off without fresh position telemetry'
            )

        x, y, floor_z = self.home_position
        self.flight_controller.set_target(
            x,
            y,
            floor_z + FLIGHT_TEST_HEIGHT_M
        )
        self.flight_started_at = time.monotonic()
        self.telemetry_lost_at = None
        self._set_state(FlightState.FLYING)
        response.success = True
        response.message = (
            f'Taking off vertically to {FLIGHT_TEST_HEIGHT_M:.2f} m'
        )
        return response

    def land_callback(self, request, response):
        """Begin a controlled landing, or disarm if takeoff never started."""
        if self.flight_state is FlightState.DISARMED:
            return self._service_failure(response, 'Drone is already disarmed')

        if self.flight_state is FlightState.READY:
            self._stop_and_disarm('Disarmed before takeoff')
        elif self.flight_state is FlightState.FLYING:
            self._begin_landing('Landing requested')

        response.success = True
        response.message = 'Landing sequence active'
        return response

    def emergency_stop_callback(self, request, response):
        """Immediately stop setpoints and request motor disarming."""
        self._stop_and_disarm('Emergency stop requested')
        response.success = True
        response.message = 'Emergency stop and disarm sent'
        return response

    def connect(self):
        if self.simulation:
            self.get_logger().info(
                'Simulation mode active; waiting for Gazebo odometry'
            )
            return

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

        self._store_position(position, received_at)

    def _on_sim_odometry(self, msg):
        position = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            float(msg.pose.pose.position.z),
        )
        self._store_position(position, time.monotonic())

    def _store_position(self, position, received_at):
        valid = all(math.isfinite(value) for value in position)

        with self.position_lock:
            self.current_position = position if valid else None
            self.position_received_at = received_at if valid else None

        if not valid or not self.context.ok():
            return

        msg = Point()
        msg.x, msg.y, msg.z = position
        self.position_pub.publish(msg)

    def _publish_sim_velocity(self, vx, vy, vz, yaw_rate):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.linear.z = vz
        msg.angular.z = math.radians(yaw_rate)
        self.sim_velocity_pub.publish(msg)

    def control_callback(self):
        with self.position_lock:
            position = self.current_position
            received_at = self.position_received_at

        msg = Twist()
        now = time.monotonic()
        position_is_fresh = (
            position is not None
            and received_at is not None
            and now - received_at <= POSITION_TIMEOUT_S
        )

        if (
            self.flight_state is FlightState.FLYING
            and now - self.flight_started_at >= MAX_FLIGHT_TIME_S
        ):
            self._begin_landing('Maximum flight time reached')

        if self.flight_state not in (FlightState.FLYING, FlightState.LANDING):
            self.velocity_debug_pub.publish(msg)
            return

        if not position_is_fresh:
            if self.telemetry_lost_at is None:
                self.telemetry_lost_at = now
                self._set_state(FlightState.LANDING)
                self.get_logger().error(
                    'Position telemetry lost; beginning blind vertical descent'
                )

            if now - self.telemetry_lost_at < TELEMETRY_LOSS_DESCENT_S:
                msg.linear.z = -LANDING_SPEED_M_S
                self.interface.send_velocity(0.0, 0.0, msg.linear.z)
            else:
                self._stop_and_disarm('Telemetry-loss descent timed out')

            self.velocity_debug_pub.publish(msg)
            return

        self.telemetry_lost_at = None

        if self.flight_state is FlightState.LANDING:
            landing_z = self.home_position[2] + LANDING_HEIGHT_M
            if position[2] <= landing_z + LANDING_COMPLETE_TOLERANCE_M:
                self._stop_and_disarm('Landing complete')
                self.velocity_debug_pub.publish(msg)
                return

        vx, vy, vz = self.flight_controller.update(position)
        if self.flight_state is FlightState.LANDING:
            vz = max(vz, -LANDING_SPEED_M_S)

        msg.linear.x = vx
        msg.linear.y = vy
        msg.linear.z = vz
        self.interface.send_velocity(vx, vy, vz)
        self.velocity_debug_pub.publish(msg)

    def _fresh_position(self):
        with self.position_lock:
            position = self.current_position
            received_at = self.position_received_at

        if (
            position is None
            or received_at is None
            or time.monotonic() - received_at > POSITION_TIMEOUT_S
        ):
            return None
        return position

    def _begin_landing(self, reason):
        if self.home_position is None:
            self._stop_and_disarm(f'{reason}; no home position')
            return

        self.flight_controller.set_target(
            self.home_position[0],
            self.home_position[1],
            self.home_position[2] + LANDING_HEIGHT_M
        )
        self._set_state(FlightState.LANDING)
        self.get_logger().warning(reason)

    def _stop_and_disarm(self, reason):
        self.interface.stop_and_disarm()
        self.flight_controller.clear_target()
        self.flight_started_at = None
        self.telemetry_lost_at = None
        self.armed_at = None
        self._set_state(FlightState.DISARMED)
        self.get_logger().warning(reason)

    def _set_state(self, state):
        self.flight_state = state
        self._publish_state()

    def _publish_state(self):
        msg = String()
        msg.data = self.flight_state.name
        self.state_pub.publish(msg)

    @staticmethod
    def _service_failure(response, message):
        response.success = False
        response.message = message
        return response

    def disconnect(self):
        self.control_timer.cancel()

        try:
            if self.flight_state is not FlightState.DISARMED:
                self._stop_and_disarm('Node disconnecting')
        finally:
            if self.telemetry is not None:
                self.telemetry.stop()
            if self.scf is not None:
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
