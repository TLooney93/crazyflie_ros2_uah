"""Project-wide runtime defaults.

Mission geometry belongs in ``crazyflie_missions/`` or ``swarm_missions/``;
swarm membership belongs in ``swarm_profiles/``.  This file contains only
settings shared by the control stack.  Units are encoded in names wherever a
value is dimensional.
"""

from pathlib import Path


# ---------------------------------------------------------------------------
# Hardware identity and cflib
# ---------------------------------------------------------------------------

DRONES = {

    # Add drones and radio addresses here.

    'cf1': 'radio://0/80/2M/E7E7E7E701',
    'cf2': 'radio://0/80/2M/E7E7E7E706',
    'cf3': 'radio://0/80/2M/E7E7E7E708',
    # 'cfw': 'radio://0/80/2M/E7E7E7E7E1',
}

CACHE_DIR = Path.home() / 'dev' / 'crazyflie' / 'cache'


# ---------------------------------------------------------------------------
# Core position controller
# ---------------------------------------------------------------------------

DEFAULT_HEIGHT = 0.5  # m in the shared world frame
MAX_VELOCITY = 0.25  # m/s, applied independently to x, y, and z
MAX_YAW_RATE = 45.0  # deg/s

# Compute and refresh each vehicle's velocity command at 50 Hz. Hardware
# transmissions are grouped into the shared dispatch frames configured below.
CONTROL_RATE_HZ = 50.0
POSITION_KP = 0.45
# Added airframe mass can create a small persistent position bias. Keep the
# integral correction slow and tightly bounded; the onboard velocity loop,
# not this outer position loop, remains responsible for carrying the mass.
POSITION_KI = 0.03
POSITION_INTEGRAL_MAX_VELOCITY_M_S = 0.08
EXTERNAL_FEEDFORWARD_TIMEOUT_S = 0.30
EXTERNAL_MAX_FEEDFORWARD_M_S = 0.12
# Swarm targets are refreshed continuously. A vehicle must stop trusting its
# retained target if the coordinator disappears.
EXTERNAL_TARGET_TIMEOUT_S = 2.0


# ---------------------------------------------------------------------------
# Telemetry and position sources
# ---------------------------------------------------------------------------

# Five onboard estimator samples per second leaves more shared-radio time for
# three vehicles' external-position updates and command packets.
TELEMETRY_PERIOD_MS = 200
# This is a conservative test-readiness threshold, not the firmware's
# critical-low cutoff. A hardware flight should begin with a charged pack so
# that voltage sag under takeoff load does not reset the radio or controller.
PREFLIGHT_MIN_BATTERY_VOLTAGE = 3.80
# Shared-radio/Vicon hardware can briefly delay ROS pose callbacks while three
# links transition into flight.  Allow short gaps without immediately forcing
# every vehicle into a blind descent; invalid/zero poses are still rejected.
POSITION_TIMEOUT_S = 2.0

# Crazyflie firmware estimator selection: 2 selects the Kalman estimator.
KALMAN_ESTIMATOR = 2
VICON_ESTIMATOR_MAX_ERROR_M = 0.15
# Limit radio traffic while retaining the full-rate ROS Vicon stream.
VICON_FORWARD_RATE_HZ = 5.0
# Reject receiver dropout sentinels and physically impossible pose jumps.
VICON_ZERO_POSITION_EPSILON_M = 1e-4
VICON_MAX_POSITION_JUMP_M = 0.35
VICON_REACQUIRE_TOLERANCE_M = 0.06
VICON_REACQUIRE_SAMPLES = 5

# One shared worker snapshots every connected vehicle's newest velocity and
# transmits the full group once per 20 ms frame. Stock Crazyflie firmware
# still requires one addressed CRTP packet per vehicle inside each frame.
RADIO_COMMAND_RATE_HZ = 50.0
# Stop refreshing a command if its controller has not renewed it. Allow a few
# timer periods for normal ROS scheduling jitter without sustaining a stale
# flight command if a controller callback stops running.
RADIO_COMMAND_TIMEOUT_S = 0.10


# ---------------------------------------------------------------------------
# Single-drone mission defaults
# ---------------------------------------------------------------------------

DEFAULT_CRAZYFLIE_MISSION = 'figure_eight'
FLIGHT_TEST_HEIGHT_M = 0.30
FLIGHT_TEST_HOVER_S = 3.0
FLIGHT_TEST_FIGURE_EIGHT_RADIUS_M = 0.35
FLIGHT_TEST_FIGURE_EIGHT_DURATION_S = 20.0
FLIGHT_TEST_POSITION_TOLERANCE_M = 0.08
ARM_SETTLE_TIME_S = 1.0
# The ROS node must not enter READY until the firmware supervisor confirms
# both armed and flyable. This bounds the blocking supervisor handshake.
ARM_CONFIRM_TIMEOUT_S = 1.0


# ---------------------------------------------------------------------------
# Shared takeoff and landing policy
# ---------------------------------------------------------------------------

# These values are consumed by CrazyflieNode for both single and swarm runs.
# Use a gentle takeoff profile for airframes carrying passive Vicon markers.
TAKEOFF_BOOST_M_S = 0.50
TAKEOFF_BOOST_END_HEIGHT_M = 0.3
# Give marker-loaded airframes a short launch-assist window. During this time
# the full configured boost may reach the onboard velocity controller and the
# non-critical stalled-climb filters do not begin accumulating. Position
# freshness, launch-column deviation, maximum flight time, and stop commands
# remain active throughout the window.
TAKEOFF_ASSIST_GRACE_S = 3.0
TAKEOFF_GRACE_MAX_VERTICAL_VELOCITY_M_S = 0.50
# During the grace window, allow the controller its normal horizontal
# authority. The launch-column deviation guard remains active independently.
TAKEOFF_GRACE_MAX_HORIZONTAL_VELOCITY_M_S = MAX_VELOCITY
# Normal takeoff cap after the launch-assist window expires.
TAKEOFF_MAX_VERTICAL_VELOCITY_M_S = 0.40
# Avoid tilting a heavy vehicle into a floor slide before it has clearance.
TAKEOFF_NEAR_FLOOR_HEIGHT_M = 0.10
TAKEOFF_NEAR_FLOOR_MAX_HORIZONTAL_VELOCITY_M_S = 0.02
TAKEOFF_MAX_HORIZONTAL_VELOCITY_M_S = 0.04
TAKEOFF_MAX_HORIZONTAL_DEVIATION_M = 0.45
# Abort a takeoff that is consuming high thrust without gaining clearance.
TAKEOFF_MIN_PROGRESS_HEIGHT_M = 0.05
TAKEOFF_PROGRESS_TIMEOUT_S = 6.0
# Marker-loaded airframes normally hover near 59-61k raw thrust.  Treat only
# sustained demand at that upper range as a stalled takeoff, and give a heavy
# vehicle enough time to clear the first 5 cm before aborting the whole swarm.
TAKEOFF_THRUST_SATURATION_THRESHOLD = 63000.0
TAKEOFF_THRUST_SATURATION_DURATION_S = 3.0
LANDING_HEIGHT_M = 0.05
LANDING_COMPLETE_TOLERANCE_M = 0.03
LANDING_SPEED_M_S = 0.10
# Do not interpret a low-altitude safety landing as an immediate touchdown.
LANDING_MIN_DURATION_S = 1.0
# Safety timeout for takeoff, hover, figure eight, and landing.
MAX_FLIGHT_TIME_S = 35.0
TELEMETRY_LOSS_DESCENT_S = 3.0


# ---------------------------------------------------------------------------
# External/terminal position targets
# ---------------------------------------------------------------------------

ENABLE_EXTERNAL_TARGETS = False

# Bounds applied whenever a ROS topic is used as the position-command source.
# Targets are absolute world coordinates, but are constrained relative to the
# position recorded when the vehicle was armed.
# The measured three-drone launch line is about 1.9 m wide. The remapped
# triangle keeps nominal travel below 0.85 m; retain modest tracking headroom.
EXTERNAL_MAX_HORIZONTAL_OFFSET_M = 0.95
EXTERNAL_MIN_HEIGHT_M = 0.10
EXTERNAL_MAX_HEIGHT_M = 2


# ---------------------------------------------------------------------------
# Swarm coordinator
# ---------------------------------------------------------------------------

DEFAULT_SWARM_PROFILE = 'two_drone_lab'
DEFAULT_SWARM_MISSION = 'two_drone_lab'
SWARM_CONTROL_RATE_HZ = 10.0
SWARM_POSITION_TOLERANCE_M = 0.07
# Small hardware vehicles do not settle at the same instant.  Takeoff only
# needs each member close enough to its commanded clearance height before the
# coordinator begins the saved mission.
SWARM_TAKEOFF_POSITION_TOLERANCE_M = 0.4
SWARM_MIN_SEPARATION_M = 0.25
SWARM_CRITICAL_SEPARATION_M = 0.12
SWARM_PREFLIGHT_MIN_SEPARATION_M = 0.25
SWARM_SEPARATION_VIOLATION_S = 1.50
# Abort an active hover/mission if any member falls well below the takeoff
# altitude for more than this short confirmation interval.
SWARM_ALTITUDE_LOSS_MARGIN_M = 0.4
SWARM_ALTITUDE_VIOLATION_S = 1.50
SWARM_MIN_AIRBORNE_HEIGHT_M = 0.05
# Rising motion from a staged member that should still be grounded indicates
# an identity mismatch or another uncommanded vehicle.
SWARM_UNCOMMANDED_MOTION_M = 0.08
SWARM_DIAGNOSTIC_PERIOD_S = 1.0
SWARM_TAKEOFF_TIMEOUT_S = 20.0
SWARM_MAX_FLIGHT_TIME_S = 90.0


# ---------------------------------------------------------------------------
# Wand input and gesture recognition
# ---------------------------------------------------------------------------

WAND_GESTURE_TOPIC = '/wand/gesture'
WAND_POSE_TOPIC = '/wand/pose'
WAND_VICON_POSE_TOPIC = '/vicon/cfw/cfw'
WAND_ENABLED_TOPIC = '/wand/enabled'
WAND_CONNECTED_TOPIC = '/wand/connected'
WAND_FRAME_ID = 'world'
WAND_SIMULATION_STEP_M = 0.05
WAND_TARGET_STEP_M = 0.05
WAND_MAX_HORIZONTAL_OFFSET_M = 0.50
WAND_TILT_TRIGGER_DEG = 20.0
WAND_TILT_REPEAT_S = 0.25
WAND_GESTURE_VERTICAL_TRAVEL_M = 0.15
WAND_GESTURE_HORIZONTAL_TRAVEL_M = 0.15
WAND_GESTURE_RETURN_TOLERANCE_M = 0.05
WAND_GESTURE_MAX_DURATION_S = 2.0
WAND_GESTURE_COOLDOWN_S = 2.0
WAND_GESTURE_BASELINE_ALPHA = 0.02

WAND_GESTURE_ACTIONS = {
    'up_down': 'takeoff',
    'side_to_side': 'start_mission',
    'down_up': 'land',
}

VALID_WAND_GESTURES = {
    'enable',
    'disable',
    'emergency_stop',
    'takeoff',
    'land',
    'move_left',
    'move_right',
    'move_forward',
    'move_backward',
    'hold',
    *WAND_GESTURE_ACTIONS,
}


def validate_config():
    """Raise ``ValueError`` for internally inconsistent safety settings."""
    errors = []
    if not DRONES:
        errors.append('DRONES must contain at least one vehicle')
    if len(set(DRONES.values())) != len(DRONES):
        errors.append('each drone must have a unique radio URI')
    if (
        CONTROL_RATE_HZ < 50.0
        or RADIO_COMMAND_RATE_HZ < 50.0
        or VICON_FORWARD_RATE_HZ <= 0.0
    ):
        errors.append(
            'control and radio command rates must be at least 50 Hz and '
            'the Vicon rate must be positive'
        )
    if RADIO_COMMAND_TIMEOUT_S < 2.0 / CONTROL_RATE_HZ:
        errors.append('radio command timeout is too short for control jitter')
    if not (
        0.0 <= POSITION_KI
        and 0.0 <= POSITION_INTEGRAL_MAX_VELOCITY_M_S <= MAX_VELOCITY
    ):
        errors.append('position integral output must fit the velocity limit')
    if EXTERNAL_TARGET_TIMEOUT_S <= 2.0 / SWARM_CONTROL_RATE_HZ:
        errors.append('external target timeout is too short for swarm jitter')
    if SWARM_UNCOMMANDED_MOTION_M <= 0.0:
        errors.append('uncommanded-motion threshold must be positive')
    if ARM_CONFIRM_TIMEOUT_S <= 0.0 or ARM_SETTLE_TIME_S < 0.0:
        errors.append('arming timeouts must be non-negative and finite')
    if not (
        0.0
        <= TAKEOFF_NEAR_FLOOR_MAX_HORIZONTAL_VELOCITY_M_S
        <= TAKEOFF_MAX_HORIZONTAL_VELOCITY_M_S
        <= TAKEOFF_GRACE_MAX_HORIZONTAL_VELOCITY_M_S
        <= MAX_VELOCITY
    ):
        errors.append('takeoff horizontal speed limits are out of order')
    if not 0.0 < TAKEOFF_MAX_VERTICAL_VELOCITY_M_S:
        errors.append('takeoff vertical speed limit must be positive')
    if not (
        0.0 <= TAKEOFF_ASSIST_GRACE_S
        and TAKEOFF_MAX_VERTICAL_VELOCITY_M_S
        <= TAKEOFF_GRACE_MAX_VERTICAL_VELOCITY_M_S
        <= 1.0
        and TAKEOFF_BOOST_M_S
        <= TAKEOFF_GRACE_MAX_VERTICAL_VELOCITY_M_S
    ):
        errors.append('takeoff assist settings are inconsistent')
    if not (
        0.0
        < SWARM_CRITICAL_SEPARATION_M
        < SWARM_MIN_SEPARATION_M
    ):
        errors.append('critical separation must be below minimum separation')
    if not 0.0 <= TAKEOFF_THRUST_SATURATION_THRESHOLD <= 65535.0:
        errors.append('takeoff thrust threshold must be a raw 16-bit value')
    if errors:
        raise ValueError(
            'Invalid cf_control configuration: ' + '; '.join(errors)
        )


validate_config()
