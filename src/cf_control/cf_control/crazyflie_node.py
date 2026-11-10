"""Per-vehicle ROS 2 controller for hardware and Gazebo flight."""

import math
import time
from threading import Lock

from cf_control.config import (
    ARM_SETTLE_TIME_S,
    CACHE_DIR,
    CONTROL_RATE_HZ,
    DEFAULT_CRAZYFLIE_MISSION,
    DRONES,
    ENABLE_EXTERNAL_TARGETS,
    EXTERNAL_FEEDFORWARD_TIMEOUT_S,
    EXTERNAL_MAX_FEEDFORWARD_M_S,
    EXTERNAL_MAX_HEIGHT_M,
    EXTERNAL_MAX_HORIZONTAL_OFFSET_M,
    EXTERNAL_MIN_HEIGHT_M,
    EXTERNAL_TARGET_TIMEOUT_S,
    FLIGHT_TEST_HEIGHT_M,
    FLIGHT_TEST_POSITION_TOLERANCE_M,
    KALMAN_ESTIMATOR,
    LANDING_COMPLETE_TOLERANCE_M,
    LANDING_HEIGHT_M,
    LANDING_MIN_DURATION_S,
    LANDING_SPEED_M_S,
    MAX_FLIGHT_TIME_S,
    MAX_VELOCITY,
    POSITION_INTEGRAL_MAX_VELOCITY_M_S,
    POSITION_KI,
    POSITION_KP,
    POSITION_TIMEOUT_S,
    PREFLIGHT_MIN_BATTERY_VOLTAGE,
    TAKEOFF_BOOST_END_HEIGHT_M,
    TAKEOFF_BOOST_M_S,
    TAKEOFF_ASSIST_GRACE_S,
    TAKEOFF_GRACE_MAX_HORIZONTAL_VELOCITY_M_S,
    TAKEOFF_GRACE_MAX_VERTICAL_VELOCITY_M_S,
    TAKEOFF_MAX_HORIZONTAL_DEVIATION_M,
    TAKEOFF_MAX_HORIZONTAL_VELOCITY_M_S,
    TAKEOFF_MAX_VERTICAL_VELOCITY_M_S,
    TAKEOFF_MIN_PROGRESS_HEIGHT_M,
    TAKEOFF_NEAR_FLOOR_HEIGHT_M,
    TAKEOFF_NEAR_FLOOR_MAX_HORIZONTAL_VELOCITY_M_S,
    TAKEOFF_PROGRESS_TIMEOUT_S,
    TAKEOFF_THRUST_SATURATION_DURATION_S,
    TAKEOFF_THRUST_SATURATION_THRESHOLD,
    TELEMETRY_LOSS_DESCENT_S,
    TELEMETRY_PERIOD_MS,
    VALID_WAND_GESTURES,
    VICON_ESTIMATOR_MAX_ERROR_M,
    VICON_FORWARD_RATE_HZ,
    VICON_MAX_POSITION_JUMP_M,
    VICON_REACQUIRE_SAMPLES,
    VICON_REACQUIRE_TOLERANCE_M,
    VICON_ZERO_POSITION_EPSILON_M,
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
from cf_control.sample_rate import SampleRateLimiter
from cf_control.takeoff import (
    TakeoffSafetyLimits,
    TakeoffSafetyMonitor,
    ground_effect_takeoff_boost,
    limit_horizontal_velocity,
    takeoff_horizontal_velocity_limit,
    takeoff_vertical_velocity_limit,
)
from cf_control.telemetry import TelemetryReader
from cf_control.wand_control import gesture_action, tilt_to_unit_offset

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger


class CrazyflieNode(Node):
    """Own one vehicle's position input, state machine, and commands."""

    def __init__(
        self,
        node_name='crazyflie_node',
        parameter_overrides=None,
        radio_dispatcher=None,
    ):
        """Create one configured hardware or simulated vehicle controller."""
        super().__init__(
            node_name,
            parameter_overrides=parameter_overrides,
        )

        self.declare_parameter('drone_id', 'cf1')
        self.declare_parameter('simulation', False)
        self.declare_parameter('positioning_source', 'vicon')
        self.declare_parameter('vicon_pose_topic', '')
        self.declare_parameter(
            'vicon_forward_rate_hz',
            VICON_FORWARD_RATE_HZ,
        )
        self.declare_parameter('wand_control_enabled', False)
        self.declare_parameter('wand_mission_gestures_enabled', True)
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
        self.declare_parameter(
            'preflight_min_battery_voltage',
            PREFLIGHT_MIN_BATTERY_VOLTAGE,
        )
        self.drone_id = self.get_parameter('drone_id').value
        self.simulation = self.get_parameter('simulation').value
        self.positioning_source = self.get_parameter(
            'positioning_source'
        ).value.lower()
        self.wand_control_enabled = bool(
            self.get_parameter('wand_control_enabled').value
        )
        self.wand_mission_gestures_enabled = bool(
            self.get_parameter('wand_mission_gestures_enabled').value
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
        self.preflight_min_battery_voltage = float(
            self.get_parameter('preflight_min_battery_voltage').value
        )
        self.vicon_forward_rate_hz = float(
            self.get_parameter('vicon_forward_rate_hz').value
        )
        self.vicon_forward_limiter = SampleRateLimiter(
            self.vicon_forward_rate_hz
        )
        self.position_publish_limiter = SampleRateLimiter(CONTROL_RATE_HZ)

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

        if (
            not math.isfinite(self.preflight_min_battery_voltage)
            or self.preflight_min_battery_voltage <= 0
        ):
            raise ValueError(
                'preflight_min_battery_voltage must be positive and finite'
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
        self.estimator_position = None
        self.estimator_position_received_at = None
        self.estimator_samples = 0
        self.battery_voltage = None
        self.battery_voltage_received_at = None
        self.attitude_degrees = None
        self.stabilizer_thrust = None
        self.position_lock = Lock()
        self.vicon_received_at = None
        self.vicon_position = None
        self.vicon_consecutive_valid_samples = 0
        self.vicon_tracking_invalid = False
        self.vicon_reacquire_position = None
        self.vicon_reacquire_samples = 0
        self.vicon_initial_samples = 0
        self.vicon_post_reset_samples = 0
        self.vicon_estimator_reset = False
        self.vicon_rate_window_started_at = time.monotonic()
        self.vicon_raw_samples = 0
        self.vicon_forwarded_samples = 0

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
                f'Vicon positioning selected: {vicon_pose_topic}; '
                f'position forwarding limited to '
                f'{self.vicon_forward_rate_hz:.1f} Hz'
            )

        self.flight_controller = FlightController(
            kp=POSITION_KP,
            ki=POSITION_KI,
            max_velocity=MAX_VELOCITY,
            integral_max_velocity=(
                POSITION_INTEGRAL_MAX_VELOCITY_M_S
            ),
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
        self.external_feedforward = (0.0, 0.0, 0.0)
        self.external_feedforward_received_at = None
        self.external_target_received_at = None
        self.telemetry_lost_at = None
        self.landing_started_at = None
        self.takeoff_safety = TakeoffSafetyMonitor(
            TakeoffSafetyLimits(
                max_horizontal_deviation_m=(
                    TAKEOFF_MAX_HORIZONTAL_DEVIATION_M
                ),
                assist_grace_s=TAKEOFF_ASSIST_GRACE_S,
                min_progress_height_m=TAKEOFF_MIN_PROGRESS_HEIGHT_M,
                progress_timeout_s=TAKEOFF_PROGRESS_TIMEOUT_S,
                thrust_saturation_threshold=(
                    TAKEOFF_THRUST_SATURATION_THRESHOLD
                ),
                thrust_saturation_duration_s=(
                    TAKEOFF_THRUST_SATURATION_DURATION_S
                ),
            )
        )
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
        self.battery_pub = self.create_publisher(
            Float32,
            f'/{self.drone_id}/battery_voltage',
            10,
        )
        self.thrust_pub = self.create_publisher(
            Float32,
            f'/{self.drone_id}/stabilizer_thrust',
            10,
        )
        self.control_phase_pub = self.create_publisher(
            String,
            f'/{self.drone_id}/control_phase',
            10,
        )

        self.target_sub = self.create_subscription(
            Point,
            f'/{self.drone_id}/target',
            self.target_callback,
            10
        )
        self.feedforward_sub = self.create_subscription(
            Twist,
            f'/{self.drone_id}/feedforward',
            self.feedforward_callback,
            10,
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
            self.interface = CrazyflieInterface(
                self.scf,
                dispatcher=radio_dispatcher,
            )
            self.telemetry = TelemetryReader(
                self.scf,
                callback=self._on_position,
                period_ms=TELEMETRY_PERIOD_MS,
                include_attitude=True,
                include_thrust=True,
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
        """Accept a bounded absolute target when external control is active."""
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
            or self.test_phase not in ('EXTERNAL_HOVER', 'WAND_HOVER')
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
        self.external_target_received_at = time.monotonic()

        if previous_target != target:
            self.get_logger().info(
                f'Target stored: {msg.x}, {msg.y}, {msg.z}'
            )

    def feedforward_callback(self, msg):
        """Store a bounded, short-lived external velocity feedforward."""
        if (
            not self.external_targets_enabled
            or self.flight_state is not FlightState.FLYING
            or self.flight_mode != 'external'
            or self.test_phase not in ('EXTERNAL_HOVER', 'WAND_HOVER')
        ):
            return

        velocity = (
            float(msg.linear.x),
            float(msg.linear.y),
            float(msg.linear.z),
        )
        if not all(math.isfinite(value) for value in velocity):
            self.get_logger().warning('Ignoring non-finite feedforward')
            return
        speed_m_s = math.sqrt(sum(value * value for value in velocity))
        if speed_m_s > EXTERNAL_MAX_FEEDFORWARD_M_S:
            self.get_logger().warning(
                'Ignoring feedforward above '
                f'{EXTERNAL_MAX_FEEDFORWARD_M_S:.2f} m/s'
            )
            return

        self.external_feedforward = velocity
        self.external_feedforward_received_at = time.monotonic()

    def clear_target_callback(self, request, response):
        """Clear the target used by the dry-run controller."""
        self.flight_controller.clear_target()
        self._clear_external_feedforward()

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

        if self.positioning_source == 'vicon' and not self.simulation:
            estimator_position = self._fresh_estimator_position()
            if estimator_position is None:
                return self._service_failure(
                    response,
                    'Cannot arm without fresh onboard estimator telemetry'
                )
            estimator_error_m = math.dist(estimator_position, position)
            if estimator_error_m > VICON_ESTIMATOR_MAX_ERROR_M:
                return self._service_failure(
                    response,
                    'Cannot arm: fused position differs from Vicon by '
                    f'{estimator_error_m:.3f} m; limit is '
                    f'{VICON_ESTIMATOR_MAX_ERROR_M:.2f} m'
                )

        if not self.simulation:
            battery_voltage = self._fresh_battery_voltage()
            if battery_voltage is None:
                return self._service_failure(
                    response,
                    'Cannot arm without fresh battery telemetry'
                )
            if battery_voltage < self.preflight_min_battery_voltage:
                return self._service_failure(
                    response,
                    f'Cannot arm: battery is {battery_voltage:.2f} V; '
                    f'require at least '
                    f'{self.preflight_min_battery_voltage:.2f} V'
                )

        self.home_position = position
        self.flight_controller.clear_target()
        try:
            supervisor_states = self.interface.arm()
        except Exception as error:
            # The request may have reached the firmware even if confirmation
            # did not. Request disarming and never advance ROS to READY.
            disarm_error = None
            try:
                self.interface.disarm()
            except Exception as cleanup_error:
                disarm_error = cleanup_error
            message = f'Firmware did not confirm arming: {error}'
            if disarm_error is not None:
                message += f'; disarm cleanup also failed: {disarm_error}'
            self.get_logger().error(message)
            return self._service_failure(response, message)

        self.armed_at = time.monotonic()
        self._set_state(FlightState.READY)
        response.success = True
        state_text = ', '.join(supervisor_states or ())
        response.message = 'Firmware confirmed armed and ready for takeoff'
        if state_text:
            response.message += f' ({state_text})'
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
        self.takeoff_safety.reset()
        self.mission_phase_index = None
        self.mission_phase_arrived_at = None
        if wand_mode:
            takeoff_phase = 'WAND_TAKEOFF'
        elif self.flight_mode == 'external':
            takeoff_phase = 'EXTERNAL_TAKEOFF'
        else:
            takeoff_phase = 'TAKEOFF'
        self._set_test_phase(takeoff_phase, self.flight_started_at)
        self.telemetry_lost_at = None
        self._set_state(FlightState.FLYING)
        self.get_logger().info(
            f'Heavy-airframe takeoff assist active for '
            f'{TAKEOFF_ASSIST_GRACE_S:.1f} s; vertical command cap '
            f'{TAKEOFF_GRACE_MAX_VERTICAL_VELOCITY_M_S:.2f} m/s'
        )
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

        action = gesture_action(gesture)

        if action == 'emergency_stop':
            response = type('Response', (), {})()
            self.emergency_stop_callback(None, response)
            self.get_logger().error('Emergency stop received from wand button')
            return

        if action == 'land':
            if self.flight_state in (FlightState.READY, FlightState.FLYING):
                response = type('Response', (), {})()
                self.land_callback(None, response)
            return

        if action == 'enable':
            if self.flight_state is FlightState.LANDING:
                self.get_logger().warning(
                    'Cannot enable wand gesture capture while landing'
                )
                return
            self.wand_active = True
            self.wand_reference_orientation = self.wand_orientation
            self.wand_last_move_at = None
            self.get_logger().info(
                'Wand gesture capture enabled; hold it level to establish '
                'neutral orientation'
            )
            return

        if action == 'disable':
            self.wand_active = False
            self.wand_reference_orientation = None
            self.wand_last_move_at = None
            self.get_logger().info(
                'Wand gesture capture disabled; active flight target retained'
            )
            return

        if not self.wand_active:
            self.get_logger().warning(
                f'Ignoring wand gesture {gesture}: wand is disabled'
            )
            return

        if action == 'takeoff':
            response = type('Response', (), {})()
            response = self._start_takeoff(response, wand_mode=True)
            if not response.success:
                self.get_logger().warning(response.message)
            else:
                self.get_logger().info(response.message)
            return

        if action == 'hold':
            self._hold_current_position('Wand hold requested')
            return

        if action == 'start_mission':
            if not self.wand_mission_gestures_enabled:
                self.get_logger().warning(
                    'Ignoring mission gesture: disabled for this launch mode'
                )
                return
            if (
                self.flight_state is not FlightState.FLYING
                or self.test_phase != 'WAND_HOVER'
            ):
                self.get_logger().warning(
                    'Ignoring mission gesture until wand hover is stable'
                )
                return
            self._set_mission_phase(0, time.monotonic())
            self.get_logger().info(
                f'Wand started mission {self.mission_name!r}'
            )
            return

        offsets = {
            'move_forward': (1.0, 0.0),
            'move_backward': (-1.0, 0.0),
            'move_left': (0.0, 1.0),
            'move_right': (0.0, -1.0),
        }
        if action in offsets:
            self._move_wand_target(*offsets[action])

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
        self._set_test_phase('WAND_HOVER', time.monotonic())
        self.get_logger().warning(reason)
        return True

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
        """Connect hardware and start telemetry, or await simulation data."""
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
        self.interface.start()
        self.telemetry.start()

    def _on_vicon_pose(self, msg):
        """Validate and forward Vicon position to the onboard estimator."""
        received_at = time.monotonic()
        self.vicon_raw_samples += 1
        position = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
        )

        if not all(math.isfinite(value) for value in position):
            self._reject_vicon_position(
                received_at,
                'non-finite Vicon position',
            )
            return

        if max(abs(value) for value in position) <= (
            VICON_ZERO_POSITION_EPSILON_M
        ):
            self._reject_vicon_position(
                received_at,
                'zero Vicon pose indicates tracking loss',
            )
            return

        if not self._vicon_jump_is_acceptable(position):
            if not self._try_reacquire_vicon(position):
                self._reject_vicon_position(
                    received_at,
                    'implausible Vicon position jump',
                )
                return

        if self.vicon_tracking_invalid:
            self.get_logger().info('Vicon position tracking recovered')
        self.vicon_tracking_invalid = False
        self.vicon_reacquire_position = None
        self.vicon_reacquire_samples = 0
        self.vicon_position = position
        self.vicon_received_at = received_at
        self.vicon_consecutive_valid_samples += 1

        # In Vicon mode the camera measurement is the authoritative position
        # for the ROS outer loop and swarm safety checks. The onboard Kalman
        # estimate remains available separately for preflight validation.
        self._store_position(position, received_at)

        should_forward = self.vicon_forward_limiter.accept(received_at)
        if should_forward:
            self.interface.send_external_position(*position)
            self.vicon_forwarded_samples += 1

        self._report_vicon_rates(received_at)

        if not should_forward:
            return

        if not self.vicon_estimator_reset:
            self.vicon_initial_samples += 1
            if self.vicon_initial_samples >= 5:
                self.get_logger().info(
                    'Vicon stream detected; resetting estimator'
                )
                self.lighthouse.reset_estimator()
                self.vicon_estimator_reset = True
                self.vicon_consecutive_valid_samples = 0
            return

        if self.vicon_post_reset_samples < 10:
            self.vicon_post_reset_samples += 1
            if self.vicon_post_reset_samples == 10:
                self.get_logger().info(
                    'Vicon positioning is ready for arming'
                )

    def _vicon_jump_is_acceptable(self, position):
        """Return whether a new measurement is close to the last valid one."""
        return (
            self.vicon_position is None
            or math.dist(position, self.vicon_position)
            <= VICON_MAX_POSITION_JUMP_M
        )

    def _try_reacquire_vicon(self, position):
        """Allow stable reacquisition only while the drone is disarmed."""
        if self.flight_state is not FlightState.DISARMED:
            self.vicon_reacquire_position = None
            self.vicon_reacquire_samples = 0
            return False

        if (
            self.vicon_reacquire_position is None
            or math.dist(position, self.vicon_reacquire_position)
            > VICON_REACQUIRE_TOLERANCE_M
        ):
            self.vicon_reacquire_position = position
            self.vicon_reacquire_samples = 1
            return False

        self.vicon_reacquire_samples += 1
        if self.vicon_reacquire_samples < VICON_REACQUIRE_SAMPLES:
            return False

        self.get_logger().warning(
            'Accepting stable Vicon position after disarmed reacquisition'
        )
        return True

    def _reject_vicon_position(self, received_at, reason):
        """Reject an invalid measurement without refreshing position age."""
        self.vicon_consecutive_valid_samples = 0
        if not self.vicon_tracking_invalid:
            self.get_logger().error(f'Rejecting {reason}')
        self.vicon_tracking_invalid = True
        self._report_vicon_rates(received_at)

    def _report_vicon_rates(self, received_at):
        """Periodically report ROS input and radio-forwarding rates."""
        elapsed = received_at - self.vicon_rate_window_started_at
        if elapsed < 5.0:
            return

        raw_rate = self.vicon_raw_samples / elapsed
        forward_rate = self.vicon_forwarded_samples / elapsed
        radio_text = ''
        if isinstance(self.interface, CrazyflieInterface):
            statistics = self.interface.consume_send_statistics()
            radio_text = (
                f'; cflib handoff velocity '
                f'{statistics["velocity"] / elapsed:.1f} Hz, '
                f'extpos '
                f'{statistics["external_position"] / elapsed:.1f} Hz'
            )
            if statistics['error']:
                radio_text += f'; send error {statistics["error"]}'
        with self.position_lock:
            estimator_rate = self.estimator_samples / elapsed
            estimator_received_at = self.estimator_position_received_at
            battery_voltage = self.battery_voltage
            attitude_degrees = self.attitude_degrees
            stabilizer_thrust = self.stabilizer_thrust
            self.estimator_samples = 0
        estimator_age = (
            received_at - estimator_received_at
            if estimator_received_at is not None
            else float('inf')
        )
        health_text = ''
        if battery_voltage is not None and attitude_degrees is not None:
            health_text = (
                f'; battery {battery_voltage:.2f} V; attitude '
                f'roll={attitude_degrees[0]:.1f}, '
                f'pitch={attitude_degrees[1]:.1f}, '
                f'yaw={attitude_degrees[2]:.1f} deg'
            )
        if stabilizer_thrust is not None:
            health_text += f'; thrust {stabilizer_thrust:.1f}'
        self.get_logger().info(
            f'Vicon rates: input {raw_rate:.1f} Hz, '
            f'forwarded {forward_rate:.1f} Hz; '
            f'estimator telemetry {estimator_rate:.1f} Hz, '
            f'age {estimator_age:.2f} s{health_text}{radio_text}'
        )
        self.vicon_rate_window_started_at = received_at
        self.vicon_raw_samples = 0
        self.vicon_forwarded_samples = 0

    def _on_position(self, data, received_at):
        position = (
            float(data['stateEstimate.x']),
            float(data['stateEstimate.y']),
            float(data['stateEstimate.z'])
        )

        valid = all(math.isfinite(value) for value in position)
        battery_voltage = float(data.get('pm.vbat', float('nan')))
        stabilizer_thrust = float(
            data.get('stabilizer.thrust', float('nan'))
        )
        attitude = tuple(
            float(data.get(name, float('nan')))
            for name in (
                'stabilizer.roll',
                'stabilizer.pitch',
                'stabilizer.yaw',
            )
        )
        with self.position_lock:
            self.estimator_position = position if valid else None
            self.estimator_position_received_at = (
                received_at if valid else None
            )
            if valid:
                self.estimator_samples += 1
            self.battery_voltage = (
                battery_voltage if math.isfinite(battery_voltage) else None
            )
            self.battery_voltage_received_at = (
                received_at if math.isfinite(battery_voltage) else None
            )
            self.attitude_degrees = (
                attitude
                if all(math.isfinite(value) for value in attitude)
                else None
            )
            self.stabilizer_thrust = (
                stabilizer_thrust
                if math.isfinite(stabilizer_thrust)
                else None
            )

        if math.isfinite(battery_voltage):
            message = Float32()
            message.data = battery_voltage
            self.battery_pub.publish(message)

        if math.isfinite(stabilizer_thrust):
            message = Float32()
            message.data = stabilizer_thrust
            self.thrust_pub.publish(message)

        if self.simulation or self.positioning_source != 'vicon':
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

        if (
            not valid
            or not self.context.ok()
            or not self.position_publish_limiter.accept(received_at)
        ):
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
        """Run one control, safety, and mission-state update."""
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
                self.landing_started_at = now
                self.test_feedforward = (0.0, 0.0, 0.0)
                self._set_state(FlightState.LANDING)
                self.get_logger().error(
                    'Position source lost; beginning blind vertical descent'
                )

            if now - self.telemetry_lost_at < TELEMETRY_LOSS_DESCENT_S:
                msg.linear.z = -LANDING_SPEED_M_S
                self.interface.send_velocity(0.0, 0.0, msg.linear.z)
            else:
                self._stop_and_disarm('Telemetry-loss descent timed out')

            self.velocity_debug_pub.publish(msg)
            return

        self.telemetry_lost_at = None

        takeoff_active = (
            self.flight_state is FlightState.FLYING
            and self.test_phase in (
                'TAKEOFF',
                'EXTERNAL_TAKEOFF',
                'WAND_TAKEOFF',
            )
        )
        if takeoff_active:
            takeoff_safety_error = self._takeoff_safety_error(position, now)
            if takeoff_safety_error is not None:
                self._begin_landing(
                    takeoff_safety_error,
                    landing_xy=position[:2],
                )
                takeoff_active = False

        if self.flight_state is FlightState.FLYING:
            self._update_test_sequence(position, now)
            takeoff_active = self.test_phase in (
                'TAKEOFF',
                'EXTERNAL_TAKEOFF',
                'WAND_TAKEOFF',
            )

        if (
            self.flight_state is FlightState.FLYING
            and self.test_phase == 'EXTERNAL_HOVER'
            and self._external_target_heartbeat_timed_out(now)
        ):
            self._begin_landing(
                'External target heartbeat timed out; coordinator lost',
                landing_xy=position[:2],
            )

        if self.flight_state is FlightState.LANDING:
            landing_z = self.home_position[2] + LANDING_HEIGHT_M
            landing_elapsed = (
                now - self.landing_started_at
                if self.landing_started_at is not None
                else 0.0
            )
            if (
                landing_elapsed >= LANDING_MIN_DURATION_S
                and position[2]
                <= landing_z + LANDING_COMPLETE_TOLERANCE_M
            ):
                self._stop_and_disarm('Landing complete')
                self.velocity_debug_pub.publish(msg)
                return

        vx, vy, vz = self.flight_controller.update(
            position,
            dt_s=1.0 / CONTROL_RATE_HZ,
        )
        vx += self.test_feedforward[0]
        vy += self.test_feedforward[1]
        vz += self.test_feedforward[2]
        if (
            self.external_feedforward_received_at is not None
            and now - self.external_feedforward_received_at
            <= EXTERNAL_FEEDFORWARD_TIMEOUT_S
            and self.test_phase == 'EXTERNAL_HOVER'
        ):
            vx += self.external_feedforward[0]
            vy += self.external_feedforward[1]
            vz += self.external_feedforward[2]
        if (
            self.flight_state is FlightState.FLYING
            and self.test_phase in (
                'TAKEOFF',
                'EXTERNAL_TAKEOFF',
                'WAND_TAKEOFF',
            )
        ):
            height_above_floor_m = position[2] - self.home_position[2]
            vz += ground_effect_takeoff_boost(
                height_above_floor_m,
                TAKEOFF_BOOST_END_HEIGHT_M,
                TAKEOFF_BOOST_M_S,
            )
        vx = max(-MAX_VELOCITY, min(vx, MAX_VELOCITY))
        vy = max(-MAX_VELOCITY, min(vy, MAX_VELOCITY))
        if takeoff_active:
            takeoff_elapsed_s = (
                now - self.flight_started_at
                if self.flight_started_at is not None
                else 0.0
            )
            vertical_velocity_limit = takeoff_vertical_velocity_limit(
                takeoff_elapsed_s,
                TAKEOFF_ASSIST_GRACE_S,
                TAKEOFF_GRACE_MAX_VERTICAL_VELOCITY_M_S,
                TAKEOFF_MAX_VERTICAL_VELOCITY_M_S,
            )
        else:
            vertical_velocity_limit = MAX_VELOCITY
        vz = max(
            -MAX_VELOCITY,
            min(vz, vertical_velocity_limit),
        )
        if takeoff_active:
            height_above_floor_m = position[2] - self.home_position[2]
            horizontal_velocity_limit = takeoff_horizontal_velocity_limit(
                takeoff_elapsed_s,
                TAKEOFF_ASSIST_GRACE_S,
                TAKEOFF_GRACE_MAX_HORIZONTAL_VELOCITY_M_S,
                height_above_floor_m,
                TAKEOFF_NEAR_FLOOR_HEIGHT_M,
                TAKEOFF_NEAR_FLOOR_MAX_HORIZONTAL_VELOCITY_M_S,
                TAKEOFF_MAX_HORIZONTAL_VELOCITY_M_S,
            )
            vx, vy = limit_horizontal_velocity(
                vx,
                vy,
                horizontal_velocity_limit,
            )
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
            if self._takeoff_height_reached(position, flight_z):
                self.flight_controller.set_target(home_x, home_y, flight_z)
                self._set_test_phase('EXTERNAL_HOVER', now)
            return

        if self.test_phase == 'EXTERNAL_HOVER':
            return

        if self.test_phase == 'WAND_TAKEOFF':
            if self._takeoff_height_reached(position, flight_z):
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
        self.external_target_received_at = None
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
        self._publish_control_phase()
        self.get_logger().info(f'Flight test phase: {phase}')

    @staticmethod
    def _near(position, target):
        return math.dist(position, target) <= FLIGHT_TEST_POSITION_TOLERANCE_M

    def _takeoff_height_reached(self, position, flight_z):
        """Require target altitude while separately enforcing launch column."""
        horizontal_deviation_m = math.hypot(
            position[0] - self.home_position[0],
            position[1] - self.home_position[1],
        )
        return (
            abs(position[2] - flight_z)
            <= FLIGHT_TEST_POSITION_TOLERANCE_M
            and horizontal_deviation_m
            <= TAKEOFF_MAX_HORIZONTAL_DEVIATION_M
        )

    def _takeoff_safety_error(self, position, now):
        """Return a reason when takeoff lacks safe position progress."""
        with self.position_lock:
            stabilizer_thrust = self.stabilizer_thrust
        return self.takeoff_safety.check(
            position=position,
            home_position=self.home_position,
            now=now,
            flight_started_at=self.flight_started_at,
            stabilizer_thrust=stabilizer_thrust,
        )

    def _external_target_heartbeat_timed_out(self, now):
        """Start the watchdog only after the coordinator's first target."""
        return (
            self.external_target_received_at is not None
            and now - self.external_target_received_at
            > EXTERNAL_TARGET_TIMEOUT_S
        )

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

    def _fresh_estimator_position(self):
        """Return recent onboard stateEstimate data for preflight checks."""
        with self.position_lock:
            position = self.estimator_position
            received_at = self.estimator_position_received_at

        if (
            position is None
            or received_at is None
            or time.monotonic() - received_at > POSITION_TIMEOUT_S
        ):
            return None
        return position

    def _fresh_battery_voltage(self):
        """Return recent hardware battery voltage for the arming gate."""
        with self.position_lock:
            voltage = self.battery_voltage
            received_at = self.battery_voltage_received_at

        if (
            voltage is None
            or received_at is None
            or time.monotonic() - received_at > POSITION_TIMEOUT_S
        ):
            return None
        return voltage

    def _positioning_source_is_ready(self):
        if self.simulation or self.positioning_source == 'lighthouse':
            return True

        return (
            self.vicon_estimator_reset
            and self.vicon_post_reset_samples >= 10
            and self.vicon_consecutive_valid_samples >= 10
            and self.vicon_received_at is not None
            and time.monotonic() - self.vicon_received_at
            <= POSITION_TIMEOUT_S
        )

    def _begin_landing(self, reason, landing_xy=None):
        if self.home_position is None:
            self._stop_and_disarm(f'{reason}; no home position')
            return

        landing_x, landing_y = (
            landing_xy
            if landing_xy is not None
            else self.home_position[:2]
        )
        self.test_feedforward = (0.0, 0.0, 0.0)
        self._clear_external_feedforward()
        self.landing_started_at = time.monotonic()
        self.takeoff_safety.reset()
        self.flight_controller.set_target(
            landing_x,
            landing_y,
            self.home_position[2] + LANDING_HEIGHT_M
        )
        self._set_state(FlightState.LANDING)
        self.get_logger().warning(reason)

    def _stop_and_disarm(self, reason):
        self.interface.stop_and_disarm()
        self.flight_controller.clear_target()
        self.flight_started_at = None
        self.test_phase = None
        self._publish_control_phase()
        self.test_phase_started_at = None
        self.mission_phase_index = None
        self.mission_phase_arrived_at = None
        self.takeoff_height_m = None
        self.test_feedforward = (0.0, 0.0, 0.0)
        self._clear_external_feedforward()
        self.external_target_received_at = None
        self.telemetry_lost_at = None
        self.landing_started_at = None
        self.takeoff_safety.reset()
        self.armed_at = None
        self.wand_active = False
        self.wand_reference_orientation = None
        self.wand_last_move_at = None
        self._set_state(FlightState.DISARMED)
        self.get_logger().warning(reason)

    def _clear_external_feedforward(self):
        """Clear any short-lived coordinator velocity feedforward."""
        self.external_feedforward = (0.0, 0.0, 0.0)
        self.external_feedforward_received_at = None

    def _set_state(self, state):
        self.flight_state = state
        self._publish_state()

    def _publish_state(self):
        msg = String()
        msg.data = self.flight_state.name
        self.state_pub.publish(msg)
        self._publish_control_phase()
        self.last_state_published_at = time.monotonic()

    def _publish_control_phase(self):
        """Publish the controller sub-state used for swarm handshakes."""
        if not hasattr(self, 'control_phase_pub'):
            return
        msg = String()
        msg.data = self.test_phase or 'IDLE'
        self.control_phase_pub.publish(msg)

    @staticmethod
    def _service_failure(response, message):
        response.success = False
        response.message = message
        return response

    def disconnect(self):
        """Stop commands, telemetry, and the underlying hardware link."""
        self.control_timer.cancel()

        try:
            if self.flight_state is not FlightState.DISARMED:
                self._stop_and_disarm('Node disconnecting')
        finally:
            if isinstance(self.interface, CrazyflieInterface):
                self.interface.shutdown()
            if self.telemetry is not None:
                self.telemetry.stop()
            if self.scf is not None:
                self.scf.close_link()


def main(args=None):
    """Run a standalone single-vehicle controller."""
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
