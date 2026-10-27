"""Configurable hardware launch for single-drone and swarm control."""

from cf_control.config import (
    DEFAULT_CRAZYFLIE_MISSION,
    DEFAULT_SWARM_MISSION,
    SWARM_MAX_FLIGHT_TIME_S,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _comma_list(value):
    return [item.strip() for item in value.split(',') if item.strip()]


def _optional_values(context, argument, drone_ids):
    values = _comma_list(LaunchConfiguration(argument).perform(context))
    if values and len(values) != len(drone_ids):
        raise ValueError(
            f'{argument} must be empty or contain one value per drone'
        )
    return values or [''] * len(drone_ids)


def _build_nodes(context):
    mode = LaunchConfiguration('mode').perform(context).strip().lower()
    input_mode = LaunchConfiguration('input').perform(context).strip().lower()
    positioning_source = LaunchConfiguration(
        'positioning_source'
    ).perform(context).strip().lower()
    requested_mission = LaunchConfiguration(
        'mission_name'
    ).perform(context).strip()
    drone_ids = _comma_list(
        LaunchConfiguration('drone_ids').perform(context)
    )

    if mode not in ('single', 'swarm'):
        raise ValueError('mode must be single or swarm')
    if input_mode not in ('autonomous', 'terminal', 'wand'):
        raise ValueError('input must be autonomous, terminal, or wand')
    if positioning_source not in ('vicon', 'lighthouse'):
        raise ValueError('positioning_source must be vicon or lighthouse')
    if not drone_ids:
        raise ValueError('drone_ids cannot be empty')
    if len(set(drone_ids)) != len(drone_ids):
        raise ValueError('drone_ids must be unique')
    if mode == 'single' and len(drone_ids) != 1:
        raise ValueError('single mode requires exactly one drone_id')
    if mode == 'swarm' and len(drone_ids) < 2:
        raise ValueError('swarm mode requires at least two drone_ids')

    mission_name = requested_mission or (
        DEFAULT_CRAZYFLIE_MISSION
        if mode == 'single'
        else DEFAULT_SWARM_MISSION
    )

    vicon_topics = _optional_values(context, 'vicon_topics', drone_ids)
    radio_uris = _optional_values(context, 'radio_uris', drone_ids)
    nodes = []

    for index, drone_id in enumerate(drone_ids):
        wand_enabled = input_mode == 'wand'
        external_enabled = input_mode == 'terminal' or (
            mode == 'swarm' and not wand_enabled
        )
        flight_mode = (
            'external'
            if external_enabled or wand_enabled
            else 'autonomous'
        )
        parameters = {
            'drone_id': drone_id,
            'simulation': False,
            'positioning_source': positioning_source,
            'wand_control_enabled': wand_enabled,
            'external_targets_enabled': external_enabled,
            'flight_mode': flight_mode,
            'max_flight_time_s': (
                SWARM_MAX_FLIGHT_TIME_S if mode == 'swarm' else 35.0
            ),
        }
        if mode == 'single':
            parameters['mission_name'] = mission_name
        if vicon_topics[index]:
            parameters['vicon_pose_topic'] = vicon_topics[index]
        if radio_uris[index]:
            parameters['radio_uri'] = radio_uris[index]

        nodes.append(
            Node(
                package='cf_control',
                executable='crazyflie_node',
                name=f'{drone_id}_control',
                parameters=[parameters],
                output='screen',
            )
        )

    if mode == 'swarm':
        nodes.append(
            Node(
                package='cf_control',
                executable='swarm_node',
                name='swarm_control',
                parameters=[{
                    'drone_ids': drone_ids,
                    'mission_name': mission_name,
                    'wand_group_mode': input_mode == 'wand',
                }],
                output='screen',
            )
        )

    return nodes


def generate_launch_description():
    """Return configurable hardware control launch actions."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='single',
            description='single or swarm',
        ),
        DeclareLaunchArgument(
            'input',
            default_value='autonomous',
            description='autonomous, terminal, or wand',
        ),
        DeclareLaunchArgument(
            'drone_ids',
            default_value='cf1',
            description='Comma-separated drone names',
        ),
        DeclareLaunchArgument(
            'positioning_source',
            default_value='vicon',
            description='vicon or lighthouse',
        ),
        DeclareLaunchArgument(
            'vicon_topics',
            default_value='',
            description='Comma-separated pose topics, one per drone',
        ),
        DeclareLaunchArgument(
            'radio_uris',
            default_value='',
            description='Optional comma-separated radio URI overrides',
        ),
        DeclareLaunchArgument(
            'mission_name',
            default_value='',
            description=(
                'Mission from crazyflie_mission.py or swarm_missions.py; '
                'empty selects the appropriate default'
            ),
        ),
        OpaqueFunction(function=_build_nodes),
    ])
