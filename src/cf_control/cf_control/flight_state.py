from enum import Enum,auto

class FlightState(Enum):
  DISARMED = auto()
  READY = auto()
  FLYING = auto()
  LANDING = auto()

