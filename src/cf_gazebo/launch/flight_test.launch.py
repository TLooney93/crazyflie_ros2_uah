import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_share = get_package_share_directory('cf_gazebo')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={
            'headless': LaunchConfiguration('headless'),
        }.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/crazyflie/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/crazyflie/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
    )

    controller = Node(
        package='cf_control',
        executable='crazyflie_node',
        name='crazyflie_sim_node',
        parameters=[{
            'drone_id': 'cf1',
            'simulation': True,
            'use_sim_time': True,
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo without its graphical interface',
        ),
        gazebo,
        bridge,
        controller,
    ])
