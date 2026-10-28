"""Two-drone spread, translation, and return demonstration."""


MISSION_NAME = 'two_drone_demo'

MISSION = [
    {
        'name': 'SPREAD',
        'hold_s': 2.0,
        'offsets': {
            'cf1': (-0.15, 0.0, 0.35),
            'cf2': (0.15, 0.0, 0.35),
        },
    },
    {
        'name': 'FORWARD',
        'hold_s': 2.0,
        'offsets': {
            'cf1': (-0.15, 0.30, 0.35),
            'cf2': (0.15, 0.30, 0.35),
        },
    },
    {
        'name': 'RETURN_HOME',
        'hold_s': 2.0,
        'offsets': {
            'cf1': (0.0, 0.0, 0.35),
            'cf2': (0.0, 0.0, 0.35),
        },
    },
]
