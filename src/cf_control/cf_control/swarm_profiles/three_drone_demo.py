"""Three-drone line-formation hardware and simulation profile."""

from cf_control.config import DRONES


PROFILE_NAME = 'three_drone_demo'

PROFILE = {
    'drone_ids': ('cf1', 'cf2', 'cf3'),
    'vicon_topics': {
        'cf1': '/vicon/cf1/cf1',
        'cf2': '/vicon/cf2/cf2',
        'cf3': '/vicon/cf3/cf3',
    },
    'radio_uris': {
        'cf1': DRONES.get('cf1', ''),
        'cf2': DRONES.get('cf2', ''),
        'cf3': DRONES.get('cf3', ''),
    },
    'mission_name': 'three_drone_demo',
    'takeoff_height_m': 0.30,
    'simulation_world': 'crazyflie_three_swarm_world.sdf',
}
