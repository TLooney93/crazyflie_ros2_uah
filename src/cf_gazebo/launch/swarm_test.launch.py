"""Launch a profile-selected Gazebo swarm and its ROS coordinator."""

import os

from ament_index_python.packages import get_package_share_directory
from cf_control.config import DEFAULT_SWARM_PROFILE
from cf_control.swarm_profiles import load_swarm_profile
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_simulation(context):
    profile_name = LaunchConfiguration(
        'profile_name'
    ).perform(context).strip()
    input_mode = LaunchConfiguration('input').perform(context).lower()
    requested_mission = LaunchConfiguration(
        'mission_name'
    ).perform(context).strip()
    if input_mode not in ('autonomous', 'terminal', 'wand'):
        raise ValueError('input must be autonomous, terminal, or wand')

    profile = load_swarm_profile(profile_name)
    drone_ids = profile['drone_ids']
    mission_name = requested_mission or profile['mission_name']
    wand_enabled = input_mode == 'wand'
    gazebo_share = get_package_share_directory('cf_gazebo')
    world = os.path.join(
        gazebo_share,
        'worlds',
        profile['simulation_world'],
    )
    if not os.path.isfile(world):
        raise ValueError(
            f'Profile {profile_name!r} simulation world was not installed: '
            f'{world}'
        )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={
            'headless': LaunchConfiguration('headless'),
            'world': world,
        }.items(),
    )

    bridge_arguments = []
    for drone_id in drone_ids:
        bridge_arguments.extend([
            f'/model/{drone_id}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            (
                f'/model/{drone_id}/odometry'
                '@nav_msgs/msg/Odometry[gz.msgs.Odometry'
            ),
        ])
    bridge_arguments.append(
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
    )
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_arguments,
        output='screen',
    )

    controllers = [
        Node(
            package='cf_control',
            executable='crazyflie_node',
            name=f'{drone_id}_sim_control',
            parameters=[{
                'drone_id': drone_id,
                'simulation': True,
                'sim_model_name': drone_id,
                'use_sim_time': True,
                'flight_mode': 'external',
                'external_targets_enabled': not wand_enabled,
                'wand_control_enabled': wand_enabled,
                'max_flight_time_s': 90.0,
            }],
            output='screen',
        )
        for drone_id in drone_ids
    ]
    coordinator = Node(
        package='cf_control',
        executable='swarm_node',
        name='swarm_sim_control',
        parameters=[{
            'drone_ids': drone_ids,
            'mission_name': mission_name,
            'wand_group_mode': wand_enabled,
            'use_sim_time': True,
        }],
        output='screen',
    )
    return [gazebo, bridge, *controllers, coordinator]


def generate_launch_description():
    """Return the profile-driven swarm simulation launch description."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo without its graphical interface',
        ),
        DeclareLaunchArgument(
            'profile_name',
            default_value=DEFAULT_SWARM_PROFILE,
            description='Named roster from cf_control/swarm_profiles/',
        ),
        DeclareLaunchArgument(
            'mission_name',
            default_value='',
            description='Optional swarm mission override',
        ),
        DeclareLaunchArgument(
            'input',
            default_value='autonomous',
            description='autonomous, terminal, or wand',
        ),
        OpaqueFunction(function=_build_simulation),
    ])
