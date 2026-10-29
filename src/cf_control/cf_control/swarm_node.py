"""ROS 2 coordinator for guarded multi-Crazyflie missions."""

import math
import time

from cf_control.config import (
    ARM_SETTLE_TIME_S,
    DEFAULT_SWARM_MISSION,
    POSITION_TIMEOUT_S,
    SWARM_CONTROL_RATE_HZ,
    SWARM_MAX_FLIGHT_TIME_S,
    SWARM_MIN_SEPARATION_M,
    SWARM_POSITION_TOLERANCE_M,
    SWARM_TAKEOFF_TIMEOUT_S,
)
from cf_control.swarm_missions import load_swarm_mission
from geometry_msgs.msg import Point
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
        self.home_positions = {}
        self.target_publishers = {}
        self.service_clients = {}
        self.subscription_handles = []

        for drone_id in self.drone_ids:
            self.target_publishers[drone_id] = self.create_publisher(
                Point,
                f'/{drone_id}/target',
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

        self.swarm_state = 'IDLE'
        self.pending_action = None
        self.pending_futures = {}
        self.action_started_at = None
        self.sequence_started_at = None
        self.run_full_sequence = False
        self.phase_index = None
        self.phase_arrived_at = None
        self.wand_flight_detected_at = None
        self.last_status_published_at = 0.0

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

        if self.pending_action is not None:
            self._check_pending_action(now)
            return

        if self.wand_group_mode:
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
            if not self._actual_positions_are_separated():
                self._safety_land('Swarm separation fell below safety limit')
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
            if self._all_at_takeoff_height():
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

    def _dispatch_action(self, action, next_state, active_only=False):
        drone_ids = self.drone_ids
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
            self._set_state('ARM_SETTLE')
        elif action == 'takeoff':
            self.action_started_at = now
            self._set_state('TAKEOFF')
        elif action == 'land':
            self._set_state('LANDING')
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

    def _all_at_takeoff_height(self):
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
            ) <= SWARM_POSITION_TOLERANCE_M
            for drone_id in self.drone_ids
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
            progress.append(f'{drone_id}={height}, state={state}')
        return '; '.join(progress)

    def _begin_mission(self):
        self.phase_index = 0
        self.phase_arrived_at = None
        self._set_state('MISSION')
        self.get_logger().info(
            f'Swarm mission phase: {self.mission[0]["name"]}'
        )

    def _update_mission(self, now):
        phase = self.mission[self.phase_index]
        targets = self._targets_for_phase(phase)
        for drone_id, target in targets.items():
            msg = Point()
            msg.x, msg.y, msg.z = target
            self.target_publishers[drone_id].publish(msg)

        all_arrived = all(
            math.dist(self.positions[drone_id], targets[drone_id])
            <= SWARM_POSITION_TOLERANCE_M
            for drone_id in self.drone_ids
        )
        if not all_arrived:
            self.phase_arrived_at = None
            return

        if self.phase_arrived_at is None:
            self.phase_arrived_at = now
            return
        if now - self.phase_arrived_at < phase['hold_s']:
            return

        self.phase_index += 1
        self.phase_arrived_at = None
        if self.phase_index >= len(self.mission):
            self._dispatch_action('land', 'LAND_REQUESTED', active_only=True)
            return

        self.get_logger().info(
            f'Swarm mission phase: {self.mission[self.phase_index]["name"]}'
        )

    def _targets_for_phase(self, phase):
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
        positions = [self.positions[drone_id] for drone_id in self.drone_ids]
        for index, first in enumerate(positions):
            for second in positions[index + 1:]:
                if math.dist(first, second) < SWARM_MIN_SEPARATION_M:
                    return False
        return True

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
            if not self._actual_positions_are_separated():
                self._safety_land(
                    'Wand swarm separation fell below safety limit'
                )
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
        self.swarm_state = state
        self._publish_status()
        self.get_logger().info(f'Swarm state: {state}')

    def _publish_periodic_status(self, now):
        if now - self.last_status_published_at >= 1.0:
            self._publish_status()

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
