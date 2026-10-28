"""Five-drone laboratory hardware and simulation profile."""

from cf_control.config import DRONES


PROFILE_NAME = 'five_drone_lab'

PROFILE = {
    'drone_ids': ('cf1', 'cf2', 'cf3', 'cf4', 'cf5'),
    'vicon_topics': {
        'cf1': '/vicon/cf231/cf231',
        'cf2': '/vicon/cf2/cf2',
        'cf3': '/vicon/cf3/cf3',
        'cf4': '/vicon/cf4/cf4',
        'cf5': '/vicon/cf5/cf5',
    },
    'radio_uris': {
        'cf1': DRONES['cf1'],
        'cf2': 'radio://1/80/2M/E7E7E7E7E6',
        # Fill these with the actual addresses after radio scanning.
        'cf3': '',
        'cf4': '',
        'cf5': '',
    },
    'mission_name': 'five_drone_demo',
    'simulation_world': 'crazyflie_five_swarm_world.sdf',
}
