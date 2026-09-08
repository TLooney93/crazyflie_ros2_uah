from pathlib import Path


# Crazyflie radio addresses
URIS = [
    'radio://0/80/2M/E7E7E7E7E7',
    'radio://0/80/2M/E7E7E7E7E6',
]


# cflib cache
CACHE_DIR = Path.home() / 'dev' / 'crazyflie' / 'cache'


# Flight parameters
DEFAULT_HEIGHT = 0.5       # meters
MAX_VELOCITY = 0.3        # m/s
MAX_YAW_RATE = 45.0       # deg/s


# Control
CONTROL_RATE_HZ = 20.0
POSITION_KP = 0.5


# Telemetry
TELEMETRY_PERIOD_MS = 100
