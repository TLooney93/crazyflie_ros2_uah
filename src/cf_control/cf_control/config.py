from pathlib import Path


# Crazyflie radio addresses each drone is mapped
# 'cf*' with its radio address as the key value pair

DRONES = {
    'cf1': 'radio://0/80/2M/E7E7E7E7E7',
    'cf2': 'radio://0/80/2M/E7E7E7E7E6',
}


# cflib cache
CACHE_DIR = Path.home() / 'dev' / 'crazyflie' / 'cache'


# **********  FLIGHT PARAMETERS  ****************

# Default target z-coordinate of 0.5 metres in our shared positioning frame.
DEFAULT_HEIGHT = 0.5       # meters
# Limit on commanded velocity. Current controller applies it separately to x, y, and z.
MAX_VELOCITY = 0.3        # m/s per axis
# Intended limit on turning speed around the vertical axis, in degrees per second.
MAX_YAW_RATE = 45.0       # deg/s


# *************  CONTROL  *****************

# Determines the intended frequency of the drone node’s control timer
CONTROL_RATE_HZ = 20.0

# proportional gain used by controller
POSITION_KP = 0.5


# *************  TELEMETRY  ***************
# sets the requested interval between position measurements
TELEMETRY_PERIOD_MS = 100
POSITION_TIMEOUT_S = 0.5

# Crazyflie firmware estimator selection: 2 selects the Kalman estimator.
KALMAN_ESTIMATOR = 2
VICON_ESTIMATOR_MAX_ERROR_M = 0.5


# ************* FIRST FLIGHT TEST ***************

# Autonomous flight-test geometry and timing.
DEFAULT_CRAZYFLIE_MISSION = 'figure_eight'
FLIGHT_TEST_HEIGHT_M = 0.30
FLIGHT_TEST_HOVER_S = 3.0
FLIGHT_TEST_FIGURE_EIGHT_RADIUS_M = 0.35
FLIGHT_TEST_FIGURE_EIGHT_DURATION_S = 20.0
FLIGHT_TEST_POSITION_TOLERANCE_M = 0.08
ARM_SETTLE_TIME_S = 1.0
LANDING_HEIGHT_M = 0.05
LANDING_COMPLETE_TOLERANCE_M = 0.03
LANDING_SPEED_M_S = 0.10
# Safety timeout for takeoff, hover, figure eight, and landing.
MAX_FLIGHT_TIME_S = 35.0
TELEMETRY_LOSS_DESCENT_S = 3.0
ENABLE_EXTERNAL_TARGETS = False

# Bounds applied whenever a ROS topic is used as the position-command source.
# Targets are absolute world coordinates, but are constrained relative to the
# position recorded when the vehicle was armed.
EXTERNAL_MAX_HORIZONTAL_OFFSET_M = 0.75
EXTERNAL_MIN_HEIGHT_M = 0.10
EXTERNAL_MAX_HEIGHT_M = 0.75


# ************* SWARM CONTROL ***************

DEFAULT_SWARM_PROFILE = 'two_drone_lab'
DEFAULT_SWARM_MISSION = 'two_drone_demo'
SWARM_CONTROL_RATE_HZ = 10.0
SWARM_POSITION_TOLERANCE_M = 0.07
SWARM_MIN_SEPARATION_M = 0.25
SWARM_TAKEOFF_TIMEOUT_S = 12.0
SWARM_MAX_FLIGHT_TIME_S = 90.0


# ************* WAND CONTROL ***************

WAND_GESTURE_TOPIC = '/wand/gesture'
WAND_POSE_TOPIC = '/wand/pose'
WAND_VICON_POSE_TOPIC = '/vicon/wand/wand'
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
    'takeoff',
    'land',
    'move_left',
    'move_right',
    'move_forward',
    'move_backward',
    'hold',
    *WAND_GESTURE_ACTIONS,
}
