from enum import auto, Enum


class FlightState(Enum):
    DISARMED = auto()
    READY = auto()
    FLYING = auto()
    LANDING = auto()
