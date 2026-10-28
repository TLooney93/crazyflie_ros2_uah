"""Two-drone laboratory hardware and simulation profile."""

from cf_control.config import DRONES


PROFILE_NAME = 'two_drone_lab'

PROFILE = {
    'drone_ids': ('cf1', 'cf2'),
    'vicon_topics': {
        'cf1': '/vicon/cf231/cf231',
        'cf2': '/vicon/cf2/cf2',
    },
    'radio_uris': {
        'cf1': DRONES['cf1'],
        # The second process normally needs a second Crazyradio.
        'cf2': 'radio://1/80/2M/E7E7E7E7E6',
    },
    'mission_name': 'two_drone_demo',
    'simulation_world': 'crazyflie_swarm_world.sdf',
}
