"""Two-drone laboratory hardware and simulation profile."""

from cf_control.config import DRONES


PROFILE_NAME = 'two_drone_lab'

PROFILE = {
    'drone_ids': ('cf1', 'cf2'),
    'vicon_topics': {
        'cf1': '/vicon/cf1/cf1',
        'cf2': '/vicon/cf2/cf2',
    },
    'radio_uris': {
        'cf1': DRONES['cf1'],
        'cf2': DRONES['cf2'],
    },
    'mission_name': 'two_drone_lab',
    'simulation_world': 'crazyflie_swarm_world.sdf',
}
