from pathlib import Path


# Crazyflie radio addresses each drone is mapped
# 'cf*' with its radio address as the key value pair

DRONES = {
    'cf1' : 'radio://0/80/2M/E7E7E7E7E7',
    'cf2' : 'radio://0/80/2M/E7E7E7E7E4',  # Physical button wand
}


# cflib cache
CACHE_DIR = Path.home() / 'dev' / 'crazyflie' / 'cache'


#**********  FLIGHT PARAMETERS  ****************

# Default target z-coordinate of 0.5 metres in our shared positioning frame.
DEFAULT_HEIGHT = 0.5       # meters
# Limit on commanded velocity. Current controller applies it separately to x, y, and z.
MAX_VELOCITY = 0.3        # m/s per axis
# Intended limit on turning speed around the vertical axis, in degrees per second.
MAX_YAW_RATE = 45.0       # deg/s


#*************  CONTROL  *****************

# Determines the intended frequency of the drone node’s control timer
CONTROL_RATE_HZ = 20.0

# proportional gain used by controller
POSITION_KP = 0.5


#*************  TELEMETRY  ***************
# sets the requested interval between position measurements
TELEMETRY_PERIOD_MS = 100
POSITION_TIMEOUT_S = 0.5


#*************  VICON  ******************

# Default ROS topic pattern for Vicon rigid-body poses. Override the
# vicon_topic parameter when the lab bridge uses a different name.
VICON_TOPIC_TEMPLATE = '/vicon/{drone_id}/pose'
VICON_FRAME_ID = 'world'
VICON_TIMEOUT_S = 0.25
VICON_READY_SAMPLES = 10
VICON_MAX_STATIONARY_SPREAD_M = 0.05


#*************  WAND INPUT  ***************

# Topic used to publish recognized wand gestures. Wand commands are kept
# separate from flight commands until their behavior and safety are verified.
WAND_GESTURE_TOPIC = '/wand/gesture'
WAND_POSE_TOPIC = '/wand/pose'
WAND_ENABLED_TOPIC = '/wand/enabled'
WAND_CONNECTED_TOPIC = '/wand/connected'
WAND_FRAME_ID = 'world'
WAND_SIMULATION_STEP_M = 0.1
WAND_TARGET_STEP_M = 0.25

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
    'emergency_stop',
}
