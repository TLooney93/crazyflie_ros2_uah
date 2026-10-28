import rclpy
import cflib.crtp
import math
import time
from threading import Lock
from rclpy.executors import ExternalShutdownException
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from rclpy.node import Node
from cf_control.flight_controller import FlightController
from cf_control.crazyflie_interface import CrazyflieInterface
from cf_control.vicon import ViconPoseValidator
from geometry_msgs.msg import Point, PoseStamped, TransformStamped, Twist
from cf_control.flight_state import FlightState
from std_srvs.srv import Trigger
from cf_control.config import (
    DRONES,
    CACHE_DIR,
    POSITION_KP,
    MAX_VELOCITY,
    CONTROL_RATE_HZ,
    POSITION_TIMEOUT_S,
    DEFAULT_HEIGHT,
    VICON_MAX_STATIONARY_SPREAD_M,
    VICON_READY_SAMPLES,
    VICON_TIMEOUT_S,
    VICON_TOPIC_TEMPLATE,
)

class CrazyflieNode(Node):

    def __init__(self):
        super().__init__('crazyflie_node')

        self.declare_parameter('drone_id', 'cf1')
        self.drone_id = self.get_parameter('drone_id').value

        if self.drone_id not in DRONES:
            raise ValueError(f"Unknown drone name: {self.drone_id}")

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

        self.declare_parameter('land_duration_s', 2.0)
        self.land_duration_s = float(
            self.get_parameter('land_duration_s').value
        )
        self.landing_timer = None
        self.position_ready = False
        self.estimator_reset = False
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

        self.declare_parameter(
            'vicon_topic',
            VICON_TOPIC_TEMPLATE.format(drone_id=self.drone_id),
        )
        self.vicon_topic = self.get_parameter('vicon_topic').value
        self.declare_parameter('vicon_message_type', 'pose_stamped')
        self.vicon_message_type = self.get_parameter(
            'vicon_message_type'
        ).value
        self.vicon_validator = ViconPoseValidator(
            sample_count=VICON_READY_SAMPLES,
            timeout_s=VICON_TIMEOUT_S,
            max_spread_m=VICON_MAX_STATIONARY_SPREAD_M,
        )
        if self.vicon_message_type == 'pose_stamped':
            self.vicon_sub = self.create_subscription(
                PoseStamped, self.vicon_topic, self.vicon_pose_callback, 10,
            )
        elif self.vicon_message_type == 'transform_stamped':
            self.vicon_sub = self.create_subscription(
                TransformStamped,
                self.vicon_topic,
                self.vicon_transform_callback,
                10,
            )
        else:
            raise ValueError(
                'vicon_message_type must be pose_stamped or transform_stamped'
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

        if msg.z < 0.15 or msg.z > 1.5:
            self.get_logger().warning(
                'Ignoring target: z must be between 0.15 and 1.50 metres'
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

        if not self.position_ready:
            response.success = False
            response.message = 'Takeoff rejected: Vicon pose is not ready.'
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

        self.flight_controller.clear_target()
        self.command_interface.release_low_level_control()
        self.command_interface.land(
            height_m=0.0,
            duration_s=self.land_duration_s,
        )
        self.landing_timer = self.create_timer(
            self.land_duration_s + 0.5,
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
        self.command_interface.disable_lighthouse_streams()
        self.command_interface.set_parameter('stabilizer.estimator', 2)
        self.command_interface.reset_estimator()
        self.estimator_reset = True
        self.get_logger().info(
            f'Waiting for stable Vicon poses on {self.vicon_topic}'
        )

    def vicon_pose_callback(self, msg):
        """Process a geometry_msgs/PoseStamped Vicon measurement."""
        self._handle_vicon_pose(msg.pose.position, msg.pose.orientation)

    def vicon_transform_callback(self, msg):
        """Process a geometry_msgs/TransformStamped Vicon measurement."""
        self._handle_vicon_pose(msg.transform.translation, msg.transform.rotation)

    def _handle_vicon_pose(self, position_msg, orientation_msg):
        """Validate, forward, and publish one Vicon pose."""
        now = time.monotonic()
        pose = self.vicon_validator.add(
            (position_msg.x, position_msg.y, position_msg.z),
            (
                orientation_msg.x,
                orientation_msg.y,
                orientation_msg.z,
                orientation_msg.w,
            ),
            now,
        )
        if pose is None:
            self.get_logger().warning('Ignoring invalid Vicon pose')
            return

        position, quaternion = pose
        self.command_interface.send_external_pose(position, quaternion)
        with self.position_lock:
            self.current_position = position
            self.position_received_at = now

        msg = Point()
        msg.x, msg.y, msg.z = position
        self.position_pub.publish(msg)

        ready = self.vicon_validator.is_ready(now)
        if ready and not self.position_ready:
            self.position_ready = True
            if self.flight_state is FlightState.DISARMED:
                self.flight_state = FlightState.READY
            self.get_logger().info('Vicon pose is stable; drone is READY')

    def control_callback(self):
        with self.position_lock:
            position = self.current_position
            received_at = self.position_received_at

        now = time.monotonic()
        if not self.vicon_validator.is_fresh(now):
            if self.position_ready:
                self.get_logger().error('Vicon pose timed out')
            self.position_ready = False
            if self.flight_state is FlightState.READY:
                self.flight_state = FlightState.DISARMED
            elif self.flight_state is FlightState.FLYING:
                self.flight_controller.clear_target()
                self.command_interface.release_low_level_control()
                self.command_interface.land(
                    height_m=0.0,
                    duration_s=self.land_duration_s,
                )
                self.flight_state = FlightState.LANDING
                self.landing_timer = self.create_timer(
                    self.land_duration_s + 0.5,
                    self.finish_landing_callback,
                )
                self.get_logger().error(
                    'Automatic landing requested after Vicon timeout'
                )

        msg = Twist()  # All components initially zero.

        if (
            self.flight_controller.target is not None
            and position is not None
            and received_at is not None
            and now - received_at <= POSITION_TIMEOUT_S
        ):
            vx, vy, vz = self.flight_controller.update(position)

            msg.linear.x = vx
            msg.linear.y = vy
            msg.linear.z = vz

            if (
                self.enable_actuation
                and self.position_ready
                and self.flight_state is FlightState.FLYING
            ):
                self.command_interface.send_velocity(vx, vy, vz)

        self.velocity_debug_pub.publish(msg)

    def disconnect(self):
        self.control_timer.cancel()
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
