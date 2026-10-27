"""Launch a two-drone Gazebo swarm and its ROS coordinator."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_control_nodes(context):
    input_mode = LaunchConfiguration('input').perform(context).lower()
    if input_mode not in ('autonomous', 'terminal', 'wand'):
        raise ValueError('input must be autonomous, terminal, or wand')

    wand_enabled = input_mode == 'wand'
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
        for drone_id in ('cf1', 'cf2')
    ]
    coordinator = Node(
        package='cf_control',
        executable='swarm_node',
        name='swarm_sim_control',
        parameters=[{
            'drone_ids': ['cf1', 'cf2'],
            'mission_name': LaunchConfiguration('mission_name'),
            'wand_group_mode': wand_enabled,
            'use_sim_time': True,
        }],
        output='screen',
    )
    return [*controllers, coordinator]


def generate_launch_description():
    gazebo_share = get_package_share_directory('cf_gazebo')
    world = os.path.join(
        gazebo_share,
        'worlds',
        'crazyflie_swarm_world.sdf',
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

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/cf1/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/cf1/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/cf2/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/cf2/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo without its graphical interface',
        ),
        DeclareLaunchArgument(
            'mission_name',
            default_value='two_drone_demo',
            description='Mission from cf_control/swarm_missions.py',
        ),
        DeclareLaunchArgument(
            'input',
            default_value='autonomous',
            description='autonomous, terminal, or wand',
        ),
        gazebo,
        bridge,
        OpaqueFunction(function=_build_control_nodes),
    ])
