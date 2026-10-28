"""Five-drone hover, shared translation, and return demonstration."""


MISSION_NAME = 'five_drone_demo'

MISSION = [
    {
        'name': 'HOVER',
        'hold_s': 2.0,
        'offsets': {
            'cf1': (0.0, 0.0, 0.35),
            'cf2': (0.0, 0.0, 0.35),
            'cf3': (0.0, 0.0, 0.35),
            'cf4': (0.0, 0.0, 0.35),
            'cf5': (0.0, 0.0, 0.35),
        },
    },
    {
        'name': 'FORWARD',
        'hold_s': 2.0,
        'offsets': {
            'cf1': (0.0, 0.25, 0.35),
            'cf2': (0.0, 0.25, 0.35),
            'cf3': (0.0, 0.25, 0.35),
            'cf4': (0.0, 0.25, 0.35),
            'cf5': (0.0, 0.25, 0.35),
        },
    },
    {
        'name': 'RETURN_HOME',
        'hold_s': 2.0,
        'offsets': {
            'cf1': (0.0, 0.0, 0.35),
            'cf2': (0.0, 0.0, 0.35),
            'cf3': (0.0, 0.0, 0.35),
            'cf4': (0.0, 0.0, 0.35),
            'cf5': (0.0, 0.0, 0.35),
        },
    },
]
