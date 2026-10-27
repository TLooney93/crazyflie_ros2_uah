import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    headless = LaunchConfiguration('headless')
    gazebo_share = get_package_share_directory('ros_gz_sim')
    simulation_share = get_package_share_directory('cf_gazebo')
    default_world = os.path.join(
        simulation_share, 'worlds', 'crazyflie_world.sdf'
    )
    world = LaunchConfiguration('world')
    model_path = os.path.join(simulation_share, 'models')

    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=(
            model_path
            + os.pathsep
            + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        ),
    )

    declare_headless = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run only the Gazebo server without the OpenGL GUI',
    )

    declare_world = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Absolute path to the Gazebo world file',
    )

    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -v 4 ', world],
            'on_exit_shutdown': 'true',
        }.items(),
        condition=UnlessCondition(headless),
    )

    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-s -r -v 4 ', world],
            'on_exit_shutdown': 'true',
        }.items(),
        condition=IfCondition(headless),
    )

    return LaunchDescription([
        declare_headless,
        declare_world,
        gazebo_resource_path,
        gazebo_gui,
        gazebo_headless,
    ])
