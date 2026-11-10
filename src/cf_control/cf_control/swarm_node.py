"""ROS 2 coordinator for guarded multi-Crazyflie missions."""

import math
import time

from cf_control.config import (
    ARM_SETTLE_TIME_S,
    DEFAULT_SWARM_MISSION,
    POSITION_TIMEOUT_S,
    SWARM_ALTITUDE_LOSS_MARGIN_M,
    SWARM_ALTITUDE_VIOLATION_S,
    SWARM_CONTROL_RATE_HZ,
    SWARM_CRITICAL_SEPARATION_M,
    SWARM_DIAGNOSTIC_PERIOD_S,
    SWARM_MAX_FLIGHT_TIME_S,
    SWARM_MIN_AIRBORNE_HEIGHT_M,
    SWARM_MIN_SEPARATION_M,
    SWARM_POSITION_TOLERANCE_M,
    SWARM_PREFLIGHT_MIN_SEPARATION_M,
    SWARM_SEPARATION_VIOLATION_S,
    SWARM_TAKEOFF_POSITION_TOLERANCE_M,
    SWARM_TAKEOFF_TIMEOUT_S,
    SWARM_UNCOMMANDED_MOTION_M,
    WAND_GESTURE_TOPIC,
)
from cf_control.swarm_missions import load_swarm_mission
from cf_control.wand_control import gesture_action
from geometry_msgs.msg import Point, Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class SwarmNode(Node):
    """Coordinate services and bounded position targets across drones."""

    def __init__(self):
        """Create subscriptions, services, and per-drone clients."""
        super().__init__('swarm_node')

        self.declare_parameter('drone_ids', ['cf1', 'cf2'])
        self.declare_parameter('mission_name', DEFAULT_SWARM_MISSION)
        self.declare_parameter('takeoff_height_m', 0.30)
        self.declare_parameter('wand_group_mode', False)

        self.drone_ids = list(self.get_parameter('drone_ids').value)
        self.mission_name = self.get_parameter('mission_name').value
        self.takeoff_height_m = float(
            self.get_parameter('takeoff_height_m').value
        )
        self.wand_group_mode = bool(
            self.get_parameter('wand_group_mode').value
        )
        self.mission = load_swarm_mission(
            self.mission_name,
            self.drone_ids,
        )

        if not math.isfinite(self.takeoff_height_m):
            raise ValueError('takeoff_height_m must be finite')
        if self.takeoff_height_m <= 0.0:
            raise ValueError('takeoff_height_m must be positive')

        self.positions = {}
        self.position_received_at = {}
        self.drone_states = {}
        self.control_phases = {}
        self.home_positions = {}
        self.target_publishers = {}
        self.feedforward_publishers = {}
        self.service_clients = {}
        self.subscription_handles = []

        for drone_id in self.drone_ids:
            self.target_publishers[drone_id] = self.create_publisher(
                Point,
                f'/{drone_id}/target',
                10,
            )
            self.feedforward_publishers[drone_id] = self.create_publisher(
                Twist,
                f'/{drone_id}/feedforward',
                10,
            )
            self.subscription_handles.append(
                self.create_subscription(
                    Point,
                    f'/{drone_id}/position',
                    lambda msg, name=drone_id: self._on_position(name, msg),
                    10,
                )
            )
            self.subscription_handles.append(
                self.create_subscription(
                    String,
                    f'/{drone_id}/control_phase',
                    lambda msg, name=drone_id: self._on_control_phase(
                        name,
                        msg,
                    ),
                    10,
                )
            )
            self.subscription_handles.append(
                self.create_subscription(
                    String,
                    f'/{drone_id}/flight_state',
                    lambda msg, name=drone_id: self._on_state(name, msg),
                    10,
                )
            )
            self.service_clients[drone_id] = {
                action: self.create_client(
                    Trigger,
                    f'/{drone_id}/{service_name}',
                )
                for action, service_name in (
                    ('arm', 'arm'),
                    ('takeoff', 'takeoff'),
                    ('land', 'land'),
                    ('stop', 'emergency_stop'),
                )
            }

        self.state_publisher = self.create_publisher(
            String,
            '/swarm/state',
            10,
        )
        self.start_service = self.create_service(
            Trigger,
            '/swarm/start',
            self.start_callback,
        )
        self.arm_service = self.create_service(
            Trigger,
            '/swarm/arm',
            self.arm_callback,
        )
        self.takeoff_service = self.create_service(
            Trigger,
            '/swarm/takeoff',
            self.takeoff_callback,
        )
        self.mission_service = self.create_service(
            Trigger,
            '/swarm/start_mission',
            self.mission_callback,
        )
        self.land_service = self.create_service(
            Trigger,
            '/swarm/land',
            self.land_callback,
        )
        self.stop_service = self.create_service(
            Trigger,
            '/swarm/emergency_stop',
            self.stop_callback,
        )
        self.wand_gesture_subscription = None
        if self.wand_group_mode:
            self.wand_gesture_subscription = self.create_subscription(
                String,
                WAND_GESTURE_TOPIC,
                self._on_wand_gesture,
                10,
            )

        self.swarm_state = 'IDLE'
        self.pending_action = None
        self.pending_futures = {}
        self.action_started_at = None
        self.sequence_started_at = None
        self.run_full_sequence = False
        self.phase_index = None
        self.phase_arrived_at = None
        self.phase_arrived_drones = set()
        self.phase_started_at = None
        self.wand_flight_detected_at = None
        self.last_status_published_at = 0.0
        self.last_diagnostic_logged_at = 0.0
        self.separation_violation_started_at = None
        self.altitude_violation_started_at = {}
        self.mission_active_drones = set()
        self.phase_transition_drones = set()
        self.staged_mission = any(
            set(phase.get('active_drones', self.drone_ids))
            != set(self.drone_ids)
            for phase in self.mission
        )

        self.control_timer = self.create_timer(
            1.0 / SWARM_CONTROL_RATE_HZ,
            self.control_callback,
        )
        self._set_state('IDLE')

    def _on_position(self, drone_id, msg):
        position = (float(msg.x), float(msg.y), float(msg.z))
        if not all(math.isfinite(value) for value in position):
            return
        self.positions[drone_id] = position
        self.position_received_at[drone_id] = time.monotonic()

    def _on_state(self, drone_id, msg):
        self.drone_states[drone_id] = msg.data

    def _on_control_phase(self, drone_id, msg):
        self.control_phases[drone_id] = msg.data

    def _on_wand_gesture(self, msg):
        """Start the saved swarm mission from a stable wand hover."""
        if gesture_action(msg.data) != 'start_mission':
            return
        if self.pending_action is not None:
            self.get_logger().warning(
                'Ignoring mission gesture while a swarm action is pending'
            )
            return
        if self.swarm_state != 'WAND_ACTIVE':
            self.get_logger().warning(
                'Ignoring mission gesture until the wand swarm is active'
            )
            return
        if not self._positions_are_fresh():
            self.get_logger().warning(
                'Ignoring mission gesture without fresh swarm positions'
            )
            return
        if not all(
            self.drone_states.get(drone_id) == 'FLYING'
            for drone_id in self.drone_ids
        ):
            self.get_logger().warning(
                'Ignoring mission gesture until every drone is flying'
            )
            return
        if not self._all_control_phases_are('WAND_HOVER'):
            self.get_logger().warning(
                'Ignoring mission gesture until every drone reaches '
                'WAND_HOVER'
            )
            return
        if not self._mission_targets_are_separated():
            self._safety_land(
                'Mission target separation is below safety limit'
            )
            return

        self.run_full_sequence = True
        self._begin_mission()
        self.get_logger().info(
            f'Wand started swarm mission {self.mission_name!r}'
        )

    def start_callback(self, request, response):
        """Queue arm, takeoff, mission, and landing as one sequence."""
        if self.wand_group_mode:
            return self._service_failure(
                response,
                'Saved missions are disabled in wand group mode',
            )
        error = self._preflight_error('arm', required_state='IDLE')
        if error:
            return self._service_failure(response, error)

        self.home_positions = dict(self.positions)
        self.run_full_sequence = True
        self.sequence_started_at = time.monotonic()
        if self.staged_mission:
            self.phase_index = 0
        self._dispatch_action('arm', 'ARMING')
        return self._service_success(
            response,
            f'Started swarm mission {self.mission_name!r}',
        )

    def arm_callback(self, request, response):
        """Arm every configured drone without starting takeoff."""
        error = self._preflight_error('arm', required_state='IDLE')
        if error:
            return self._service_failure(response, error)

        self.home_positions = dict(self.positions)
        self.run_full_sequence = False
        self.sequence_started_at = time.monotonic()
        self._dispatch_action('arm', 'ARMING')
        return self._service_success(response, 'Swarm arm request queued')

    def takeoff_callback(self, request, response):
        """Take off every armed drone and hold at the configured height."""
        if self.wand_group_mode:
            return self._service_failure(
                response,
                'Use the wand takeoff gesture in wand group mode',
            )
        error = self._action_error('takeoff', required_state='READY')
        if error:
            return self._service_failure(response, error)

        self.run_full_sequence = False
        self._dispatch_action('takeoff', 'TAKEOFF_REQUESTED')
        return self._service_success(response, 'Swarm takeoff request queued')

    def mission_callback(self, request, response):
        """Run the selected mission from a stable swarm hover."""
        if self.wand_group_mode:
            return self._service_failure(
                response,
                'Saved missions are disabled in wand group mode',
            )
        if self.swarm_state != 'HOVER':
            return self._service_failure(
                response,
                f'Cannot start mission from {self.swarm_state}',
            )
        if not self._positions_are_fresh():
            return self._service_failure(
                response,
                'Cannot start mission without fresh positions',
            )
        if not self._mission_targets_are_separated():
            return self._service_failure(
                response,
                'Mission target separation is below the safety limit',
            )

        self._begin_mission()
        return self._service_success(response, 'Swarm mission started')

    def land_callback(self, request, response):
        """Request a controlled landing from every active drone."""
        if self.pending_action is not None:
            return self._service_failure(response, 'A swarm action is pending')
        if self.swarm_state == 'IDLE' and self._all_drones_disarmed():
            return self._service_failure(response, 'Swarm is already idle')
        if not self._services_are_ready('land'):
            return self._service_failure(
                response,
                'Land services are not ready',
            )

        self.run_full_sequence = False
        self._dispatch_action('land', 'LAND_REQUESTED', active_only=True)
        return self._service_success(response, 'Swarm landing request queued')

    def stop_callback(self, request, response):
        """Immediately request stop and disarm for every drone."""
        if not self._services_are_ready('stop'):
            return self._service_failure(
                response,
                'Emergency-stop services are not ready',
            )

        self.run_full_sequence = False
        self._dispatch_action('stop', 'STOPPING')
        return self._service_success(response, 'Swarm emergency stop queued')

    def control_callback(self):
        """Advance pending service actions and the active mission phase."""
        now = time.monotonic()
        self._publish_periodic_status(now)
        self._log_flight_diagnostics(now)

        if self.staged_mission and self.run_full_sequence:
            self._publish_staged_transition_heartbeat()

        if self.pending_action is not None:
            self._check_pending_action(now)
            return

        if self.staged_mission and self.run_full_sequence:
            motion_error = self._uncommanded_motion_error()
            if motion_error:
                self.get_logger().error(motion_error)
                self.run_full_sequence = False
                self._dispatch_action('stop', 'STOPPING')
                return
            if self._update_staged_transition(now):
                return

        if self.wand_group_mode and self.swarm_state != 'MISSION':
            self._update_wand_group_monitor(now)
            return

        if self.swarm_state == 'ARM_SETTLE':
            if now - self.action_started_at >= ARM_SETTLE_TIME_S:
                if self.run_full_sequence:
                    self._dispatch_action('takeoff', 'TAKEOFF_REQUESTED')
                else:
                    self._set_state('READY')
            return

        if self.swarm_state in ('TAKEOFF', 'HOVER', 'MISSION'):
            if not self._positions_are_fresh():
                self._safety_land('Swarm position data became stale')
                return
            aborted = [
                drone_id
                for drone_id in (
                    self.mission_active_drones
                    if self.staged_mission
                    else self.drone_ids
                )
                if self.drone_states.get(drone_id)
                in ('LANDING', 'DISARMED')
            ]
            if aborted:
                self._safety_land(
                    'Swarm member aborted active flight: '
                    + ', '.join(aborted)
                )
                return
            if self.swarm_state in ('HOVER', 'MISSION'):
                altitude_error = self._altitude_safety_error(now)
                if altitude_error:
                    self._safety_land(altitude_error)
                    return
            separation_error = self._separation_safety_error(now)
            if separation_error:
                self._safety_land(separation_error)
                return
            if (
                self.sequence_started_at is not None
                and now - self.sequence_started_at > SWARM_MAX_FLIGHT_TIME_S
            ):
                self._safety_land('Swarm maximum flight time reached')
                return

        if self.swarm_state == 'TAKEOFF':
            if now - self.action_started_at > SWARM_TAKEOFF_TIMEOUT_S:
                self._safety_land(
                    'Swarm takeoff timed out: '
                    + self._takeoff_progress_summary()
                )
                return
            if (
                self._all_at_takeoff_height()
                and self._all_control_phases_are('EXTERNAL_HOVER')
            ):
                self._set_state('HOVER')
                if self.run_full_sequence:
                    if self._mission_targets_are_separated():
                        self._begin_mission()
                    else:
                        self._safety_land(
                            'Mission target separation is below safety limit'
                        )
            return

        if self.swarm_state == 'MISSION':
            self._update_mission(now)
            return

        if self.swarm_state in ('LANDING', 'STOPPING'):
            if self._all_drones_disarmed():
                self.run_full_sequence = False
                self._set_state('IDLE')

    def _preflight_error(self, action, required_state):
        error = self._action_error(action, required_state)
        if error:
            return error
        if not self._positions_are_fresh():
            return 'Fresh positions have not been received for every drone'
        if any(
            self.drone_states.get(drone_id) != 'DISARMED'
            for drone_id in self.drone_ids
        ):
            return 'Every drone must report DISARMED before arming'
        separation = self._minimum_separation()
        if (
            separation is None
            or separation[0] < SWARM_PREFLIGHT_MIN_SEPARATION_M
        ):
            measured = (
                'unknown'
                if separation is None
                else f'{separation[0]:.3f} m'
            )
            return (
                'Preflight separation is '
                f'{measured}; require at least '
                f'{SWARM_PREFLIGHT_MIN_SEPARATION_M:.2f} m'
            )
        return None

    def _action_error(self, action, required_state):
        if self.swarm_state != required_state:
            return f'Cannot {action} from {self.swarm_state}'
        if self.pending_action is not None:
            return 'A swarm action is already pending'
        if not self._services_are_ready(action):
            return f'Not every {action} service is ready'
        return None

    def _services_are_ready(self, action):
        return all(
            self.service_clients[drone_id][action].service_is_ready()
            for drone_id in self.drone_ids
        )

    def _dispatch_action(
        self, action, next_state, active_only=False, drone_ids=None,
    ):
        drone_ids = list(
            self.drone_ids if drone_ids is None else drone_ids
        )
        if active_only:
            drone_ids = [
                drone_id
                for drone_id in self.drone_ids
                if self.drone_states.get(drone_id) != 'DISARMED'
            ]

        self.pending_action = action
        self.pending_futures = {
            drone_id: self.service_clients[drone_id][action].call_async(
                Trigger.Request()
            )
            for drone_id in drone_ids
        }
        self.action_started_at = time.monotonic()
        self._set_state(next_state)

        if not self.pending_futures:
            self.pending_action = None
            self._set_state('IDLE')

    def _check_pending_action(self, now):
        if not all(future.done() for future in self.pending_futures.values()):
            return

        action = self.pending_action
        failures = []
        for drone_id, future in self.pending_futures.items():
            try:
                result = future.result()
                if result is None or not result.success:
                    message = result.message if result else 'no response'
                    failures.append(f'{drone_id}: {message}')
            except Exception as error:  # ROS future surfaces transport errors.
                failures.append(f'{drone_id}: {error}')

        self.pending_action = None
        self.pending_futures = {}
        if failures:
            self._action_failed(action, failures)
            return

        if action == 'arm':
            self.action_started_at = now
            self._set_state(
                'PHASE_ARM_SETTLE'
                if self.staged_mission and self.run_full_sequence
                else 'ARM_SETTLE'
            )
        elif action == 'takeoff':
            self.action_started_at = now
            self._set_state(
                'PHASE_TAKEOFF'
                if self.staged_mission and self.run_full_sequence
                else 'TAKEOFF'
            )
        elif action == 'land':
            self._set_state(
                'PHASE_LANDING'
                if self.staged_mission and self.run_full_sequence
                else 'LANDING'
            )
        elif action == 'stop':
            self._set_state('STOPPING')

    def _action_failed(self, action, failures):
        detail = '; '.join(failures)
        self.get_logger().error(f'Swarm {action} failed: {detail}')
        self.run_full_sequence = False
        if action != 'stop' and self._services_are_ready('stop'):
            self._dispatch_action('stop', 'STOPPING')
        else:
            self._set_state('ERROR')

    def _all_at_takeoff_height(self, drone_ids=None):
        if not self.home_positions:
            return False
        return all(
            self.drone_states.get(drone_id) == 'FLYING'
            and math.dist(
                self.positions[drone_id],
                (
                    self.home_positions[drone_id][0],
                    self.home_positions[drone_id][1],
                    self.home_positions[drone_id][2]
                    + self.takeoff_height_m,
                ),
            ) <= SWARM_TAKEOFF_POSITION_TOLERANCE_M
            for drone_id in (drone_ids or self.drone_ids)
        )

    def _all_control_phases_are(self, expected, drone_ids=None):
        """Return whether every controller acknowledged the same sub-state."""
        return all(
            self.control_phases.get(drone_id) == expected
            for drone_id in (drone_ids or self.drone_ids)
        )

    def _takeoff_progress_summary(self):
        """Describe each member's measured height when takeoff times out."""
        progress = []
        for drone_id in self.drone_ids:
            position = self.positions.get(drone_id)
            home = self.home_positions.get(drone_id)
            if position is None or home is None:
                height = 'unknown'
            else:
                height = f'{position[2] - home[2]:.2f} m'
            state = self.drone_states.get(drone_id, 'unknown')
            control_phase = self.control_phases.get(drone_id, 'unknown')
            progress.append(
                f'{drone_id}={height}, state={state}, phase={control_phase}'
            )
        return '; '.join(progress)

    def _transition_to_mission_phase(self):
        """Land, arm, and launch only the members requested by a phase."""
        if self.phase_index >= len(self.mission):
            cleanup_drones = {
                item for item in self.drone_ids
                if self.drone_states.get(item) != 'DISARMED'
            }
            if cleanup_drones:
                self.phase_transition_drones = cleanup_drones
                self._dispatch_action(
                    'land', 'PHASE_LANDING',
                    drone_ids=self.phase_transition_drones,
                )
            else:
                self.run_full_sequence = False
                self._set_state('IDLE')
            return

        desired = set(self.mission[self.phase_index]['active_drones'])
        to_land = self.mission_active_drones - desired
        if to_land:
            self.phase_transition_drones = to_land
            self._dispatch_action(
                'land', 'PHASE_LANDING', drone_ids=to_land,
            )
            return

        to_launch = desired - self.mission_active_drones
        if to_launch:
            ready = {
                item for item in to_launch
                if self.drone_states.get(item) == 'READY'
            }
            needs_arm = to_launch - ready
            self.phase_transition_drones = to_launch
            if needs_arm:
                self.phase_transition_drones = needs_arm
                self._dispatch_action(
                    'arm', 'PHASE_ARMING', drone_ids=needs_arm,
                )
            else:
                self._dispatch_action(
                    'takeoff', 'PHASE_TAKEOFF', drone_ids=ready,
                )
            return

        self._start_current_mission_phase(desired)

    def _uncommanded_motion_error(self):
        """Reject motion by a staged member that should remain grounded."""
        active = self.mission_active_drones | self.phase_transition_drones
        for drone_id in self.drone_ids:
            if drone_id in active:
                continue
            position = self.positions.get(drone_id)
            home = self.home_positions.get(drone_id)
            if position is None or home is None:
                continue
            rise_m = position[2] - home[2]
            if rise_m > SWARM_UNCOMMANDED_MOTION_M:
                return (
                    f'Uncommanded motion: inactive {drone_id} rose '
                    f'{rise_m:.3f} m; possible radio/Vicon identity mismatch'
                )
        return None

    def _publish_staged_transition_heartbeat(self):
        """Keep unchanged airborne members controlled during transitions."""
        if self.swarm_state not in {
            'PHASE_ARMING',
            'PHASE_ARM_SETTLE',
            'PHASE_TAKEOFF',
            'PHASE_LANDING',
        }:
            return
        if self.phase_index is None or self.phase_index >= len(self.mission):
            return

        retained_drones = (
            self.mission_active_drones - self.phase_transition_drones
        )
        if not retained_drones:
            return

        phase = self.mission[self.phase_index]
        if phase.get('trajectory') is not None:
            targets, _ = self._trajectory_commands(phase, 0.0)
        else:
            targets = self._targets_for_phase(phase)
        targets = {
            drone_id: target
            for drone_id, target in targets.items()
            if drone_id in retained_drones
        }
        feedforwards = {
            drone_id: (0.0, 0.0, 0.0)
            for drone_id in targets
        }
        self._publish_motion_commands(targets, feedforwards)

    def _update_staged_transition(self, now):
        """Advance a phase-level land/arm/takeoff transition."""
        if self.swarm_state == 'PHASE_LANDING':
            if all(
                self.drone_states.get(item) == 'DISARMED'
                for item in self.phase_transition_drones
            ):
                self.mission_active_drones -= self.phase_transition_drones
                self.phase_transition_drones = set()
                self._transition_to_mission_phase()
            return True
        if self.swarm_state == 'PHASE_ARM_SETTLE':
            if now - self.action_started_at >= ARM_SETTLE_TIME_S:
                self.phase_transition_drones = set()
                self._transition_to_mission_phase()
            return True
        if self.swarm_state == 'PHASE_TAKEOFF':
            if now - self.action_started_at > SWARM_TAKEOFF_TIMEOUT_S:
                self._safety_land('Mission phase takeoff timed out')
                return True
            if (
                self._all_at_takeoff_height(self.phase_transition_drones)
                and self._all_control_phases_are(
                    'EXTERNAL_HOVER', self.phase_transition_drones,
                )
            ):
                self.mission_active_drones |= self.phase_transition_drones
                self.phase_transition_drones = set()
                desired = set(
                    self.mission[self.phase_index]['active_drones']
                )
                self._start_current_mission_phase(desired)
            return True
        return False

    def _start_current_mission_phase(self, active_drones):
        self.mission_active_drones = set(active_drones)
        self.phase_arrived_at = None
        self.phase_arrived_drones = set()
        self.phase_started_at = time.monotonic()
        self._set_state('MISSION')
        self.get_logger().info(
            f'Swarm mission phase: {self.mission[self.phase_index]["name"]}'
        )

    def _begin_mission(self):
        now = time.monotonic()
        self.phase_index = 0
        self.phase_arrived_at = None
        self.phase_arrived_drones = set()
        self.phase_started_at = now
        self._set_state('MISSION')
        self.get_logger().info(
            f'Swarm mission phase: {self.mission[0]["name"]}'
        )

    def _update_mission(self, now):
        phase = self.mission[self.phase_index]
        elapsed_s = now - self.phase_started_at
        if phase.get('trajectory') is not None:
            targets, feedforwards = self._trajectory_commands(
                phase,
                elapsed_s,
            )
        else:
            targets = self._targets_for_phase(phase)
            if self.staged_mission:
                targets = {
                    item: target for item, target in targets.items()
                    if item in self.mission_active_drones
                }
            feedforwards = {
                drone_id: (0.0, 0.0, 0.0)
                for drone_id in targets
            }
        self._publish_motion_commands(targets, feedforwards)

        timeout_s = phase.get('timeout_s')
        if (
            timeout_s is not None
            and self.phase_started_at is not None
            and now - self.phase_started_at >= timeout_s
        ):
            errors = ', '.join(
                f'{drone_id}='
                f'{math.dist(self.positions[drone_id], target):.3f} m'
                for drone_id, target in targets.items()
            )
            self._safety_land(
                f'Swarm mission phase {phase["name"]} timed out after '
                f'{timeout_s:.1f} s; target errors: {errors}'
            )
            return

        trajectory = phase.get('trajectory')
        if trajectory is not None:
            if elapsed_s >= trajectory['duration_s']:
                self._advance_mission_phase(now)
            return

        arrival_tolerance_m = phase.get(
            'arrival_tolerance_m',
            SWARM_POSITION_TOLERANCE_M,
        )
        target_errors = {
            drone_id: math.dist(
                self.positions[drone_id],
                targets[drone_id],
            )
            for drone_id in targets
        }
        for drone_id, error_m in target_errors.items():
            if error_m <= arrival_tolerance_m:
                self.phase_arrived_drones.add(drone_id)
            elif error_m > arrival_tolerance_m * 1.5:
                self.phase_arrived_drones.discard(drone_id)
        if len(self.phase_arrived_drones) != len(targets):
            self.phase_arrived_at = None
            return

        if self.phase_arrived_at is None:
            self.phase_arrived_at = now
            return
        if now - self.phase_arrived_at < phase['hold_s']:
            return

        self._advance_mission_phase(now)

    def _advance_mission_phase(self, now):
        """Advance to the next phase or request the automatic landing."""
        self.phase_index += 1
        self.phase_arrived_at = None
        self.phase_arrived_drones = set()
        if self.phase_index >= len(self.mission):
            self.phase_started_at = None
            if self.staged_mission:
                self._transition_to_mission_phase()
            else:
                self._dispatch_action(
                    'land', 'LAND_REQUESTED', active_only=True,
                )
            return

        if self.staged_mission:
            self._transition_to_mission_phase()
            return
        self.phase_started_at = now
        self.get_logger().info(
            f'Swarm mission phase: {self.mission[self.phase_index]["name"]}'
        )

    def _publish_motion_commands(self, targets, feedforwards):
        """Publish synchronized position and velocity-feedforward commands."""
        for drone_id, target in targets.items():
            target_msg = Point()
            target_msg.x, target_msg.y, target_msg.z = target
            self.target_publishers[drone_id].publish(target_msg)

            feedforward_msg = Twist()
            (
                feedforward_msg.linear.x,
                feedforward_msg.linear.y,
                feedforward_msg.linear.z,
            ) = feedforwards[drone_id]
            self.feedforward_publishers[drone_id].publish(feedforward_msg)

    def _formation_frame(self):
        """Return the measured launch-line center and horizontal basis."""
        first_id = self.drone_ids[0]
        last_id = self.drone_ids[-1]
        first_home = self.home_positions[first_id]
        last_home = self.home_positions[last_id]
        dx = last_home[0] - first_home[0]
        dy = last_home[1] - first_home[1]
        baseline_m = math.hypot(dx, dy)
        if baseline_m <= 1e-6:
            raise ValueError('Formation baseline is too small')
        along = (dx / baseline_m, dy / baseline_m)
        across = (-along[1], along[0])
        center = (
            sum(self.home_positions[item][0] for item in self.drone_ids)
            / len(self.drone_ids),
            sum(self.home_positions[item][1] for item in self.drone_ids)
            / len(self.drone_ids),
        )
        return center, along, across

    def _formation_targets_to_world(self, formation_targets):
        """Resolve midpoint-relative formation coordinates into world poses."""
        center, along, across = self._formation_frame()
        return {
            drone_id: (
                center[0]
                + coordinates[0] * along[0]
                + coordinates[1] * across[0],
                center[1]
                + coordinates[0] * along[1]
                + coordinates[1] * across[1],
                self.home_positions[drone_id][2] + coordinates[2],
            )
            for drone_id, coordinates in formation_targets.items()
        }

    def _formation_velocities_to_world(self, formation_velocities):
        """Rotate formation-frame velocity vectors into the world frame."""
        _, along, across = self._formation_frame()
        return {
            drone_id: (
                velocity[0] * along[0] + velocity[1] * across[0],
                velocity[0] * along[1] + velocity[1] * across[1],
                velocity[2],
            )
            for drone_id, velocity in formation_velocities.items()
        }

    def _trajectory_commands(self, phase, elapsed_s):
        """Return continuous formation targets and velocity feedforwards."""
        trajectory = phase['trajectory']
        duration_s = trajectory['duration_s']
        progress = max(0.0, min(elapsed_s / duration_s, 1.0))
        rotation_angle = math.radians(trajectory.get('angle_deg', 360.0))
        angle = rotation_angle * progress
        angular_rate = rotation_angle / duration_s
        radius_m = trajectory['radius_m']
        height_m = trajectory['height_m']
        phase_step = 2.0 * math.pi / len(self.drone_ids)
        formation_targets = {}
        formation_velocities = {}
        for index, drone_id in enumerate(self.drone_ids):
            drone_angle = math.pi + index * phase_step + angle
            formation_targets[drone_id] = (
                radius_m * math.cos(drone_angle),
                radius_m * math.sin(drone_angle),
                height_m,
            )
            formation_velocities[drone_id] = (
                -radius_m * angular_rate * math.sin(drone_angle),
                radius_m * angular_rate * math.cos(drone_angle),
                0.0,
            )
        return (
            self._formation_targets_to_world(formation_targets),
            self._formation_velocities_to_world(formation_velocities),
        )

    def _targets_for_phase(self, phase):
        if phase.get('trajectory') is not None:
            return self._trajectory_commands(phase, 0.0)[0]
        formation_targets = phase.get('formation_targets')
        if formation_targets is not None:
            return self._formation_targets_to_world(formation_targets)

        return {
            drone_id: tuple(
                home_value + offset_value
                for home_value, offset_value in zip(
                    self.home_positions[drone_id],
                    phase['offsets'][drone_id],
                )
            )
            for drone_id in self.drone_ids
        }

    def _mission_targets_are_separated(self):
        if not self.home_positions:
            return False
        for phase in self.mission:
            targets = list(self._targets_for_phase(phase).values())
            for index, first in enumerate(targets):
                for second in targets[index + 1:]:
                    if math.dist(first, second) < SWARM_MIN_SEPARATION_M:
                        return False
        return True

    def _positions_are_fresh(self):
        now = time.monotonic()
        return all(
            drone_id in self.positions
            and now - self.position_received_at[drone_id]
            <= POSITION_TIMEOUT_S
            for drone_id in self.drone_ids
        )

    def _all_drones_disarmed(self):
        return all(
            self.drone_states.get(drone_id) == 'DISARMED'
            for drone_id in self.drone_ids
        )

    def _actual_positions_are_separated(self):
        separation = self._minimum_separation()
        return (
            separation is not None
            and separation[0] >= SWARM_MIN_SEPARATION_M
        )

    def _minimum_separation(self):
        """Return minimum measured separation and its drone pair."""
        closest = None
        for index, first_id in enumerate(self.drone_ids):
            if first_id not in self.positions:
                return None
            for second_id in self.drone_ids[index + 1:]:
                if second_id not in self.positions:
                    return None
                distance = math.dist(
                    self.positions[first_id],
                    self.positions[second_id],
                )
                if closest is None or distance < closest[0]:
                    closest = (distance, first_id, second_id)
        return closest

    def _separation_safety_error(self, now):
        """Return a safety error for critical or sustained proximity."""
        separation = self._minimum_separation()
        if separation is None:
            self.separation_violation_started_at = None
            return None

        distance, first_id, second_id = separation
        if distance < SWARM_CRITICAL_SEPARATION_M:
            self.separation_violation_started_at = None
            return (
                f'Critical swarm separation: {first_id}/{second_id} '
                f'at {distance:.3f} m'
            )

        if distance >= SWARM_MIN_SEPARATION_M:
            self.separation_violation_started_at = None
            return None

        if self.separation_violation_started_at is None:
            self.separation_violation_started_at = now
            self.get_logger().warning(
                f'Separation warning: {first_id}/{second_id} at '
                f'{distance:.3f} m'
            )
            return None

        violation_s = now - self.separation_violation_started_at
        if violation_s < SWARM_SEPARATION_VIOLATION_S:
            return None
        return (
            f'Sustained swarm separation violation: {first_id}/{second_id} '
            f'at {distance:.3f} m for {violation_s:.2f} s'
        )

    def _altitude_safety_error(self, now):
        """Detect a sustained collapse after takeoff has completed."""
        minimum_height_m = max(
            SWARM_MIN_AIRBORNE_HEIGHT_M,
            self.takeoff_height_m - SWARM_ALTITUDE_LOSS_MARGIN_M,
        )
        monitored_drones = (
            self.mission_active_drones
            if (
                getattr(self, 'staged_mission', False)
                and self.swarm_state == 'MISSION'
            )
            else self.drone_ids
        )
        for drone_id in monitored_drones:
            position = self.positions.get(drone_id)
            home = self.home_positions.get(drone_id)
            if position is None or home is None:
                self.altitude_violation_started_at.pop(drone_id, None)
                continue
            height_m = position[2] - home[2]
            if height_m >= minimum_height_m:
                self.altitude_violation_started_at.pop(drone_id, None)
                continue
            started_at = self.altitude_violation_started_at.setdefault(
                drone_id,
                now,
            )
            violation_s = now - started_at
            if violation_s >= SWARM_ALTITUDE_VIOLATION_S:
                return (
                    f'Sustained altitude loss: {drone_id} at '
                    f'{height_m:.3f} m above home; require at least '
                    f'{minimum_height_m:.2f} m'
                )
        return None

    def _update_wand_group_monitor(self, now):
        """Monitor common wand control without issuing movement targets."""
        if self._all_drones_disarmed():
            self.wand_flight_detected_at = None
            if self.swarm_state != 'IDLE':
                self._set_state('IDLE')
            return

        if self.swarm_state in ('LANDING', 'STOPPING'):
            return

        states = [self.drone_states.get(name) for name in self.drone_ids]
        if any(state == 'LANDING' for state in states):
            self._set_state('LANDING')
            return

        if any(state == 'FLYING' for state in states):
            if self.wand_flight_detected_at is None:
                self.wand_flight_detected_at = now
            if not self._positions_are_fresh():
                self._safety_land('Wand swarm position data became stale')
                return
            if self._all_control_phases_are('WAND_HOVER'):
                altitude_error = self._altitude_safety_error(now)
                if altitude_error:
                    self._safety_land(f'Wand {altitude_error}')
                    return
            separation_error = self._separation_safety_error(now)
            if separation_error:
                self._safety_land(f'Wand {separation_error}')
                return
            if (
                now - self.wand_flight_detected_at >= 1.0
                and not all(state == 'FLYING' for state in states)
            ):
                self._safety_land(
                    'Not every wand swarm member entered flight'
                )
                return
            if self.swarm_state != 'WAND_ACTIVE':
                self._set_state('WAND_ACTIVE')
            return

        if any(state == 'READY' for state in states):
            if self.swarm_state != 'WAND_READY':
                self._set_state('WAND_READY')

    def _safety_land(self, reason):
        self.get_logger().error(reason)
        self.run_full_sequence = False
        if self._services_are_ready('land'):
            self._dispatch_action('land', 'LAND_REQUESTED', active_only=True)
        elif self._services_are_ready('stop'):
            self._dispatch_action('stop', 'STOPPING')
        else:
            self._set_state('ERROR')

    def _set_state(self, state):
        if state in ('IDLE', 'TAKEOFF'):
            self.altitude_violation_started_at.clear()
        self.swarm_state = state
        self._publish_status()
        self.get_logger().info(f'Swarm state: {state}')

    def _publish_periodic_status(self, now):
        if now - self.last_status_published_at >= 1.0:
            self._publish_status()

    def _log_flight_diagnostics(self, now):
        """Log measured positions, targets, and closest separation."""
        if (
            self.swarm_state not in ('TAKEOFF', 'HOVER', 'MISSION')
            or now - self.last_diagnostic_logged_at
            < SWARM_DIAGNOSTIC_PERIOD_S
        ):
            return

        position_text = ', '.join(
            f'{drone_id}=('
            f'{self.positions[drone_id][0]:.3f},'
            f'{self.positions[drone_id][1]:.3f},'
            f'{self.positions[drone_id][2]:.3f})'
            for drone_id in self.drone_ids
            if drone_id in self.positions
        )
        targets = self._active_diagnostic_targets(now)
        target_text = ', '.join(
            f'{drone_id}=('
            f'{targets[drone_id][0]:.3f},'
            f'{targets[drone_id][1]:.3f},'
            f'{targets[drone_id][2]:.3f})'
            for drone_id in self.drone_ids
            if drone_id in targets
        )
        separation = self._minimum_separation()
        separation_text = (
            'unknown'
            if separation is None
            else (
                f'{separation[0]:.3f} m '
                f'({separation[1]}/{separation[2]})'
            )
        )
        self.get_logger().info(
            f'Swarm diagnostics: positions[{position_text}]; '
            f'targets[{target_text}]; '
            f'min separation={separation_text}'
        )
        self.last_diagnostic_logged_at = now

    def _active_diagnostic_targets(self, now):
        """Return the targets expected for the current swarm state."""
        if not self.home_positions:
            return {}
        if self.swarm_state in ('TAKEOFF', 'HOVER'):
            return {
                drone_id: (
                    self.home_positions[drone_id][0],
                    self.home_positions[drone_id][1],
                    self.home_positions[drone_id][2]
                    + self.takeoff_height_m,
                )
                for drone_id in self.drone_ids
            }
        if self.swarm_state == 'MISSION' and self.phase_index is not None:
            phase = self.mission[self.phase_index]
            if phase.get('trajectory') is not None:
                elapsed_s = now - self.phase_started_at
                return self._trajectory_commands(phase, elapsed_s)[0]
            return self._targets_for_phase(phase)
        return {}

    def _publish_status(self):
        msg = String()
        msg.data = self.swarm_state
        self.state_publisher.publish(msg)
        self.last_status_published_at = time.monotonic()

    @staticmethod
    def _service_success(response, message):
        response.success = True
        response.message = message
        return response

    @staticmethod
    def _service_failure(response, message):
        response.success = False
        response.message = message
        return response


def main(args=None):
    """Run the swarm coordinator until ROS shuts down."""
    rclpy.init(args=args)
    node = None
    try:
        node = SwarmNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except (KeyboardInterrupt, ExternalShutdownException):
                pass
        try:
            rclpy.try_shutdown()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
