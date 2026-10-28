import math
from threading import Lock
import time

from cf_control.config import (
    ARM_SETTLE_TIME_S,
    CACHE_DIR,
    CONTROL_RATE_HZ,
    DEFAULT_CRAZYFLIE_MISSION,
    DRONES,
    ENABLE_EXTERNAL_TARGETS,
    EXTERNAL_MAX_HEIGHT_M,
    EXTERNAL_MAX_HORIZONTAL_OFFSET_M,
    EXTERNAL_MIN_HEIGHT_M,
    FLIGHT_TEST_HEIGHT_M,
    FLIGHT_TEST_POSITION_TOLERANCE_M,
    KALMAN_ESTIMATOR,
    LANDING_COMPLETE_TOLERANCE_M,
    LANDING_HEIGHT_M,
    LANDING_SPEED_M_S,
    MAX_FLIGHT_TIME_S,
    MAX_VELOCITY,
    POSITION_KP,
    POSITION_TIMEOUT_S,
    TELEMETRY_LOSS_DESCENT_S,
    TELEMETRY_PERIOD_MS,
    VALID_WAND_GESTURES,
    VICON_ESTIMATOR_MAX_ERROR_M,
    WAND_GESTURE_TOPIC,
    WAND_MAX_HORIZONTAL_OFFSET_M,
    WAND_POSE_TOPIC,
    WAND_TARGET_STEP_M,
    WAND_TILT_REPEAT_S,
    WAND_TILT_TRIGGER_DEG,
)
from cf_control.crazyflie_interface import (
    CrazyflieInterface,
    SimulatedCrazyflieInterface,
)
from cf_control.crazyflie_missions import load_crazyflie_mission
from cf_control.flight_controller import FlightController
from cf_control.flight_state import FlightState
from cf_control.lighthouse import LighthouseManager
from cf_control.telemetry import TelemetryReader
from cf_control.wand_control import tilt_to_unit_offset
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
import cflib.crtp
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class CrazyflieNode(Node):

    def __init__(self):
        super().__init__('crazyflie_node')

        self.declare_parameter('drone_id', 'cf1')
        self.declare_parameter('simulation', False)
        self.declare_parameter('positioning_source', 'vicon')
        self.declare_parameter('vicon_pose_topic', '')
        self.declare_parameter('wand_control_enabled', False)
        self.declare_parameter(
            'external_targets_enabled',
            ENABLE_EXTERNAL_TARGETS,
        )
        self.declare_parameter('flight_mode', 'autonomous')
        self.declare_parameter('radio_uri', '')
        self.declare_parameter('sim_model_name', 'crazyflie')
        self.declare_parameter('mission_name', DEFAULT_CRAZYFLIE_MISSION)
        self.declare_parameter('flight_height_m', FLIGHT_TEST_HEIGHT_M)
        self.declare_parameter('max_flight_time_s', MAX_FLIGHT_TIME_S)
        self.drone_id = self.get_parameter('drone_id').value
        self.simulation = self.get_parameter('simulation').value
        self.positioning_source = self.get_parameter(
            'positioning_source'
        ).value.lower()
        self.wand_control_enabled = bool(
            self.get_parameter('wand_control_enabled').value
        )
        self.external_targets_enabled = bool(
            self.get_parameter('external_targets_enabled').value
        )
        self.flight_mode = self.get_parameter('flight_mode').value.lower()
        self.sim_model_name = self.get_parameter('sim_model_name').value
        self.mission_name = self.get_parameter('mission_name').value.strip()
        self.flight_height_m = float(
            self.get_parameter('flight_height_m').value
        )
        self.max_flight_time_s = float(
            self.get_parameter('max_flight_time_s').value
        )

        if self.positioning_source not in ('lighthouse', 'vicon'):
            raise ValueError(
                'positioning_source must be "lighthouse" or "vicon"'
            )

        if self.flight_mode not in ('autonomous', 'external'):
            raise ValueError('flight_mode must be "autonomous" or "external"')

        self.mission_phases = load_crazyflie_mission(self.mission_name)

        if (
            not math.isfinite(self.flight_height_m)
            or self.flight_height_m <= 0
        ):
            raise ValueError(
                'flight_height_m must be a positive finite number'
            )

        if (
            not math.isfinite(self.max_flight_time_s)
            or self.max_flight_time_s <= 0
        ):
            raise ValueError(
                'max_flight_time_s must be a positive finite number'
            )

        radio_uri = self.get_parameter('radio_uri').value
        if (
            not self.simulation
            and not radio_uri
            and self.drone_id not in DRONES
        ):
            raise ValueError(
                f'Unknown drone name {self.drone_id!r}; provide radio_uri'
            )
        self.uri = radio_uri or DRONES.get(self.drone_id, '')

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
        self.vicon_received_at = None
        self.vicon_position = None
        self.vicon_initial_samples = 0
        self.vicon_post_reset_samples = 0
        self.vicon_estimator_reset = False

        self.vicon_pose_sub = None
        if not self.simulation and self.positioning_source == 'vicon':
            vicon_pose_topic = self.get_parameter('vicon_pose_topic').value
            if not vicon_pose_topic:
                vicon_pose_topic = (
                    f'/vicon/{self.drone_id}/{self.drone_id}'
                )
            self.vicon_pose_sub = self.create_subscription(
                PoseStamped,
                vicon_pose_topic,
                self._on_vicon_pose,
                10,
            )
            self.get_logger().info(
                f'Vicon positioning selected: {vicon_pose_topic}'
            )

        self.flight_controller = FlightController(
                    kp=POSITION_KP,
                    max_velocity=MAX_VELOCITY
                )

        self.flight_state = FlightState.DISARMED
        self.home_position = None
        self.armed_at = None
        self.flight_started_at = None
        self.test_phase = None
        self.test_phase_started_at = None
        self.mission_phase_index = None
        self.mission_phase_arrived_at = None
        self.takeoff_height_m = None
        self.test_feedforward = (0.0, 0.0, 0.0)
        self.telemetry_lost_at = None
        self.wand_active = False
        self.wand_orientation = None
        self.wand_reference_orientation = None
        self.wand_pose_received_at = None
        self.wand_last_move_at = None
        self.last_state_published_at = 0.0

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

        self.wand_gesture_sub = None
        self.wand_pose_sub = None
        if self.wand_control_enabled:
            self.wand_gesture_sub = self.create_subscription(
                String,
                WAND_GESTURE_TOPIC,
                self.wand_gesture_callback,
                10,
            )
            self.wand_pose_sub = self.create_subscription(
                PoseStamped,
                WAND_POSE_TOPIC,
                self.wand_pose_callback,
                10,
            )
            self.get_logger().info(
                'Wand control available; manual enable is still required'
            )

        if self.simulation:
            velocity_topic = f'/model/{self.sim_model_name}/cmd_vel'
            odometry_topic = f'/model/{self.sim_model_name}/odometry'
            self.sim_velocity_pub = self.create_publisher(
                Twist,
                velocity_topic,
                10,
            )
            self.sim_odometry_sub = self.create_subscription(
                Odometry,
                odometry_topic,
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
        if self.flight_mode == 'autonomous':
            self.get_logger().info(
                f'Autonomous mission selected: {self.mission_name}'
            )

    def target_callback(self, msg):
        if not self.external_targets_enabled:
            self.get_logger().warning(
                'Ignoring external target: this launch mode disabled them'
            )
            return

        if self.flight_state is not FlightState.FLYING:
            self.get_logger().warning(
                'Ignoring target: drone is not in FLYING state'
            )
            return

        if (
            self.flight_mode != 'external'
            or self.test_phase != 'EXTERNAL_HOVER'
        ):
            self.get_logger().warning(
                'Ignoring target: external command mode is not ready'
            )
            return

        target = (msg.x, msg.y, msg.z)

        if not all(math.isfinite(value) for value in target):
            self.get_logger().warning(
                'Ignoring target: coordinates must be finite'
            )
            return

        if self.home_position is None:
            self.get_logger().warning('Ignoring target: home is not recorded')
            return

        home_x, home_y, floor_z = self.home_position
        horizontal_offset = math.hypot(msg.x - home_x, msg.y - home_y)
        relative_height = msg.z - floor_z
        if horizontal_offset > EXTERNAL_MAX_HORIZONTAL_OFFSET_M:
            self.get_logger().warning(
                'Ignoring target: horizontal offset exceeds '
                f'{EXTERNAL_MAX_HORIZONTAL_OFFSET_M:.2f} m'
            )
            return
        if not (
            EXTERNAL_MIN_HEIGHT_M
            <= relative_height
            <= EXTERNAL_MAX_HEIGHT_M
        ):
            self.get_logger().warning(
                'Ignoring target: height must be between '
                f'{EXTERNAL_MIN_HEIGHT_M:.2f} and '
                f'{EXTERNAL_MAX_HEIGHT_M:.2f} m above home'
            )
            return

        previous_target = self.flight_controller.target
        self.flight_controller.set_target(
            msg.x,
            msg.y,
            msg.z
        )

        if previous_target != target:
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

        if not self._positioning_source_is_ready():
            return self._service_failure(
                response,
                f'{self.positioning_source} positioning is not ready'
            )

        position = self._fresh_position()
        if position is None:
            return self._service_failure(
                response,
                'Cannot arm without fresh position telemetry'
            )

        if (
            self.positioning_source == 'vicon'
            and not self.simulation
            and (
                self.vicon_position is None
                or math.dist(position, self.vicon_position)
                > VICON_ESTIMATOR_MAX_ERROR_M
            )
        ):
            return self._service_failure(
                response,
                'Cannot arm: fused position does not agree with Vicon'
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
        """Start either the configured autonomous or external flight."""
        return self._start_takeoff(response, wand_mode=False)

    def _start_takeoff(self, response, wand_mode):
        """Start either the autonomous test or a wand-controlled flight."""
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
        takeoff_height_m = self.flight_height_m
        if not wand_mode and self.flight_mode == 'autonomous':
            first_phase = self.mission_phases[0]
            first_offset = (
                first_phase['offset']
                if first_phase['kind'] == 'waypoint'
                else first_phase['center_offset']
            )
            takeoff_height_m = first_offset[2]
        self.takeoff_height_m = takeoff_height_m
        self.flight_controller.set_target(
            x,
            y,
            floor_z + takeoff_height_m
        )
        self.flight_started_at = time.monotonic()
        self.mission_phase_index = None
        self.mission_phase_arrived_at = None
        if wand_mode:
            self.test_phase = 'WAND_TAKEOFF'
        elif self.flight_mode == 'external':
            self.test_phase = 'EXTERNAL_TAKEOFF'
        else:
            self.test_phase = 'TAKEOFF'
        self.test_phase_started_at = self.flight_started_at
        self.telemetry_lost_at = None
        self._set_state(FlightState.FLYING)
        response.success = True
        if wand_mode:
            response.message = (
                f'Taking off to {takeoff_height_m:.2f} m for wand control'
            )
        elif self.flight_mode == 'external':
            response.message = (
                f'Taking off to {takeoff_height_m:.2f} m; '
                'waiting for external targets'
            )
        else:
            response.message = (
                f'Taking off to {takeoff_height_m:.2f} m for mission '
                f'{self.mission_name}'
            )
        return response

    def wand_gesture_callback(self, msg):
        """Apply a guarded command from the wand command topic."""
        gesture = msg.data.strip().lower().replace(' ', '_')
        if gesture not in VALID_WAND_GESTURES:
            self.get_logger().warning(
                f'Ignoring unsupported wand gesture: {msg.data!r}'
            )
            return

        if gesture == 'land':
            if self.flight_state in (FlightState.READY, FlightState.FLYING):
                response = type('Response', (), {})()
                self.land_callback(None, response)
            return

        if gesture == 'enable':
            if self.flight_state is FlightState.LANDING:
                self.get_logger().warning(
                    'Cannot enable wand control while landing'
                )
                return
            self.wand_active = True
            self.wand_reference_orientation = self.wand_orientation
            self.wand_last_move_at = None
            if self.flight_state is FlightState.FLYING:
                self._enter_wand_hover('Wand control enabled')
            self.get_logger().info(
                'Wand enabled; hold it level to establish neutral orientation'
            )
            return

        if gesture == 'disable':
            self.wand_active = False
            self.wand_reference_orientation = None
            if self.flight_state is FlightState.FLYING:
                self._hold_current_position('Wand disabled; holding position')
            return

        if not self.wand_active:
            self.get_logger().warning(
                f'Ignoring wand gesture {gesture}: wand is disabled'
            )
            return

        if gesture == 'takeoff':
            response = type('Response', (), {})()
            response = self._start_takeoff(response, wand_mode=True)
            if not response.success:
                self.get_logger().warning(response.message)
            else:
                self.get_logger().info(response.message)
            return

        if gesture == 'hold':
            self._hold_current_position('Wand hold requested')
            return

        offsets = {
            'move_forward': (1.0, 0.0),
            'move_backward': (-1.0, 0.0),
            'move_left': (0.0, 1.0),
            'move_right': (0.0, -1.0),
        }
        if gesture in offsets:
            self._move_wand_target(*offsets[gesture])

    def wand_pose_callback(self, msg):
        """Use physical-wand tilt as bounded horizontal movement input."""
        orientation = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )
        try:
            # Validation is performed by the conversion helper.
            tilt_to_unit_offset(orientation, orientation, 180.0)
        except ValueError as error:
            self.get_logger().warning(f'Ignoring wand pose: {error}')
            return

        now = time.monotonic()
        self.wand_orientation = orientation
        self.wand_pose_received_at = now
        if not self.wand_active:
            return
        if self.wand_reference_orientation is None:
            self.wand_reference_orientation = orientation
            self.get_logger().info('Wand neutral orientation calibrated')
            return
        if (
            self.flight_state is not FlightState.FLYING
            or self.test_phase != 'WAND_HOVER'
        ):
            return
        if (
            self.wand_last_move_at is not None
            and now - self.wand_last_move_at < WAND_TILT_REPEAT_S
        ):
            return

        offset = tilt_to_unit_offset(
            self.wand_reference_orientation,
            orientation,
            WAND_TILT_TRIGGER_DEG,
        )
        if offset is not None and self._move_wand_target(*offset):
            self.wand_last_move_at = now

    def _move_wand_target(self, dx_unit, dy_unit):
        """Move the current target one bounded step in the XY plane."""
        if (
            not self.wand_active
            or self.flight_state is not FlightState.FLYING
            or self.test_phase != 'WAND_HOVER'
        ):
            self.get_logger().warning(
                'Ignoring wand movement until wand takeoff reaches hover'
            )
            return False

        position = self._fresh_position()
        if position is None or self.home_position is None:
            self.get_logger().warning(
                'Ignoring wand movement without fresh position telemetry'
            )
            return False

        current_target = self.flight_controller.target or position
        home_x, home_y, floor_z = self.home_position
        requested_x = current_target[0] + dx_unit * WAND_TARGET_STEP_M
        requested_y = current_target[1] + dy_unit * WAND_TARGET_STEP_M
        offset_x = requested_x - home_x
        offset_y = requested_y - home_y
        distance = math.hypot(offset_x, offset_y)
        if distance > WAND_MAX_HORIZONTAL_OFFSET_M:
            scale = WAND_MAX_HORIZONTAL_OFFSET_M / distance
            requested_x = home_x + offset_x * scale
            requested_y = home_y + offset_y * scale

        self.flight_controller.set_target(
            requested_x,
            requested_y,
            floor_z + self.flight_height_m,
        )
        return True

    def _hold_current_position(self, reason):
        """Replace the active target with the current measured position."""
        position = self._fresh_position()
        if position is None or self.flight_state is not FlightState.FLYING:
            return False
        self.test_feedforward = (0.0, 0.0, 0.0)
        self.flight_controller.set_target(*position)
        self.test_phase = 'WAND_HOVER'
        self.get_logger().warning(reason)
        return True

    def _enter_wand_hover(self, reason):
        """Stop the autonomous path and hold before accepting wand motion."""
        if self._hold_current_position(reason):
            self.test_phase = 'WAND_HOVER'

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

        self.interface.set_parameter(
            'stabilizer.estimator',
            KALMAN_ESTIMATOR,
        )
        self.get_logger().info('Kalman estimator selected')

        if self.positioning_source == 'lighthouse':
            self.interface.set_parameter('lighthouse.fwdToEstimator', 1)
            self.get_logger().info('Requesting estimator reset')
            self.lighthouse.reset_estimator()

            self.get_logger().info(
                'Checking Lighthouse input and estimator convergence'
            )
            self.lighthouse.verify(timeout_s=10.0)

            self.get_logger().info(
                'Lighthouse input and variance stability checks passed'
            )
        else:
            self.interface.set_parameter('lighthouse.fwdToEstimator', 0)
            self.get_logger().info(
                'Lighthouse estimator forwarding disabled; '
                'waiting for Vicon poses'
            )
        self.telemetry.start()

    def _on_vicon_pose(self, msg):
        """Validate and forward Vicon position to the onboard estimator."""
        position = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
        )

        if not all(math.isfinite(value) for value in position):
            self.get_logger().warning('Ignoring non-finite Vicon position')
            return

        self.interface.send_external_position(*position)
        self.vicon_position = position
        self.vicon_received_at = time.monotonic()

        if not self.vicon_estimator_reset:
            self.vicon_initial_samples += 1
            if self.vicon_initial_samples >= 5:
                self.get_logger().info(
                    'Vicon stream detected; resetting estimator'
                )
                self.lighthouse.reset_estimator()
                self.vicon_estimator_reset = True
            return

        if self.vicon_post_reset_samples < 10:
            self.vicon_post_reset_samples += 1
            if self.vicon_post_reset_samples == 10:
                self.get_logger().info(
                    'Vicon positioning is ready for arming'
                )

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
        if now - self.last_state_published_at >= 1.0:
            self._publish_state()
        position_is_fresh = (
            position is not None
            and received_at is not None
            and now - received_at <= POSITION_TIMEOUT_S
            and self._positioning_source_is_ready()
        )

        if (
            self.flight_state is FlightState.FLYING
            and now - self.flight_started_at >= self.max_flight_time_s
        ):
            self._begin_landing('Maximum flight time reached')

        if self.flight_state not in (FlightState.FLYING, FlightState.LANDING):
            self.velocity_debug_pub.publish(msg)
            return

        if not position_is_fresh:
            if self.telemetry_lost_at is None:
                self.telemetry_lost_at = now
                self.test_feedforward = (0.0, 0.0, 0.0)
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

        if self.flight_state is FlightState.FLYING:
            self._update_test_sequence(position, now)

        if self.flight_state is FlightState.LANDING:
            landing_z = self.home_position[2] + LANDING_HEIGHT_M
            if position[2] <= landing_z + LANDING_COMPLETE_TOLERANCE_M:
                self._stop_and_disarm('Landing complete')
                self.velocity_debug_pub.publish(msg)
                return

        vx, vy, vz = self.flight_controller.update(position)
        vx += self.test_feedforward[0]
        vy += self.test_feedforward[1]
        vz += self.test_feedforward[2]
        vx = max(-MAX_VELOCITY, min(vx, MAX_VELOCITY))
        vy = max(-MAX_VELOCITY, min(vy, MAX_VELOCITY))
        vz = max(-MAX_VELOCITY, min(vz, MAX_VELOCITY))
        if self.flight_state is FlightState.LANDING:
            vz = max(vz, -LANDING_SPEED_M_S)

        msg.linear.x = vx
        msg.linear.y = vy
        msg.linear.z = vz
        self.interface.send_velocity(vx, vy, vz)
        self.velocity_debug_pub.publish(msg)

    def _update_test_sequence(self, position, now):
        """Advance the configured autonomous, external, or wand flight."""
        home_x, home_y, floor_z = self.home_position
        takeoff_height_m = self.takeoff_height_m or self.flight_height_m
        flight_z = floor_z + takeoff_height_m

        if self.test_phase == 'EXTERNAL_TAKEOFF':
            if self._near(position, (home_x, home_y, flight_z)):
                self.flight_controller.set_target(home_x, home_y, flight_z)
                self._set_test_phase('EXTERNAL_HOVER', now)
            return

        if self.test_phase == 'EXTERNAL_HOVER':
            return

        if self.test_phase == 'WAND_TAKEOFF':
            if self._near(position, (home_x, home_y, flight_z)):
                self.flight_controller.set_target(home_x, home_y, flight_z)
                self._set_test_phase('WAND_HOVER', now)
            return

        if self.test_phase == 'WAND_HOVER':
            return

        if self.test_phase == 'TAKEOFF':
            if self._near(position, (home_x, home_y, flight_z)):
                self._set_mission_phase(0, now)
            return

        if self.mission_phase_index is None:
            return

        phase = self.mission_phases[self.mission_phase_index]
        if phase['kind'] == 'waypoint':
            offset = phase['offset']
            target = (
                home_x + offset[0],
                home_y + offset[1],
                floor_z + offset[2],
            )
            self.test_feedforward = (0.0, 0.0, 0.0)
            self.flight_controller.set_target(*target)

            if not self._near(position, target):
                self.mission_phase_arrived_at = None
                return

            if self.mission_phase_arrived_at is None:
                self.mission_phase_arrived_at = now
            if now - self.mission_phase_arrived_at >= phase['hold_s']:
                self._advance_mission(now)
            return

        if phase['kind'] == 'figure_eight':
            elapsed = now - self.test_phase_started_at
            duration = phase['duration_s']
            progress = min(elapsed / duration, 1.0)

            # Smoothstep makes angular velocity zero at entry and exit.
            eased = progress * progress * (3.0 - 2.0 * progress)
            angle = 2.0 * math.pi * eased
            angular_rate = (
                2.0
                * math.pi
                * 6.0
                * progress
                * (1.0 - progress)
                / duration
            )
            radius = phase['radius_m']
            center = phase['center_offset']
            center_x = home_x + center[0]
            center_y = home_y + center[1]
            center_z = floor_z + center[2]

            self.flight_controller.set_target(
                center_x + radius * math.sin(angle),
                center_y + radius * math.sin(angle) * math.cos(angle),
                center_z,
            )
            self.test_feedforward = (
                radius * angular_rate * math.cos(angle),
                radius * angular_rate * math.cos(2.0 * angle),
                0.0,
            )

            if progress >= 1.0:
                self.test_feedforward = (0.0, 0.0, 0.0)
                self.flight_controller.set_target(
                    center_x,
                    center_y,
                    center_z,
                )
                self._advance_mission(now)
            return

    def _set_mission_phase(self, index, now):
        """Enter one validated phase from the selected mission."""
        self.mission_phase_index = index
        self.mission_phase_arrived_at = None
        self.test_feedforward = (0.0, 0.0, 0.0)
        self._set_test_phase(self.mission_phases[index]['name'], now)

    def _advance_mission(self, now):
        """Advance to the next phase or begin the shared landing sequence."""
        next_index = self.mission_phase_index + 1
        if next_index >= len(self.mission_phases):
            self._begin_landing(
                f'Mission {self.mission_name} complete; landing at origin'
            )
            return
        self._set_mission_phase(next_index, now)

    def _set_test_phase(self, phase, now):
        self.test_phase = phase
        self.test_phase_started_at = now
        self.get_logger().info(f'Flight test phase: {phase}')

    @staticmethod
    def _near(position, target):
        return math.dist(position, target) <= FLIGHT_TEST_POSITION_TOLERANCE_M

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

    def _positioning_source_is_ready(self):
        if self.simulation or self.positioning_source == 'lighthouse':
            return True

        return (
            self.vicon_estimator_reset
            and self.vicon_post_reset_samples >= 10
            and self.vicon_received_at is not None
            and time.monotonic() - self.vicon_received_at
            <= POSITION_TIMEOUT_S
        )

    def _begin_landing(self, reason):
        if self.home_position is None:
            self._stop_and_disarm(f'{reason}; no home position')
            return

        self.test_feedforward = (0.0, 0.0, 0.0)
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
        self.test_phase = None
        self.test_phase_started_at = None
        self.mission_phase_index = None
        self.mission_phase_arrived_at = None
        self.takeoff_height_m = None
        self.test_feedforward = (0.0, 0.0, 0.0)
        self.telemetry_lost_at = None
        self.armed_at = None
        self.wand_active = False
        self.wand_reference_orientation = None
        self.wand_last_move_at = None
        self._set_state(FlightState.DISARMED)
        self.get_logger().warning(reason)

    def _set_state(self, state):
        self.flight_state = state
        self._publish_state()

    def _publish_state(self):
        msg = String()
        msg.data = self.flight_state.name
        self.state_pub.publish(msg)
        self.last_state_published_at = time.monotonic()

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
        if node is not None:
            try:
                node.disconnect()
            except (KeyboardInterrupt, ExternalShutdownException):
                pass
            try:
                node.destroy_node()
            except (KeyboardInterrupt, ExternalShutdownException):
                pass
        try:
            rclpy.try_shutdown()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
