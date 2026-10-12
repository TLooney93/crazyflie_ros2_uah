from pathlib import Path


# Crazyflie radio addresses each drone is mapped
# 'cf*' with its radio address as the key value pair

DRONES = {
    'cf1' : 'radio://0/80/2M/E7E7E7E7E7',
    'cf2' : 'radio://0/80/2M/E7E7E7E7E6',
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


# ************* FIRST FLIGHT TEST ***************

# The first test is intentionally limited to a vertical takeoff and landing.
FLIGHT_TEST_HEIGHT_M = 0.30
ARM_SETTLE_TIME_S = 1.0
LANDING_HEIGHT_M = 0.05
LANDING_COMPLETE_TOLERANCE_M = 0.03
LANDING_SPEED_M_S = 0.10
MAX_FLIGHT_TIME_S = 10.0
TELEMETRY_LOSS_DESCENT_S = 3.0
ENABLE_EXTERNAL_TARGETS = False
