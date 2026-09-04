import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    gazebo_share = get_package_share_directory('ros_gz_sim')
    simulation_share = get_package_share_directory('cf_gazebo')
    world = os.path.join(
        simulation_share, 'worlds', 'crazyflie_world.sdf'
    )
    model_path = os.path.join(simulation_share, 'models')

    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=model_path + os.pathsep + os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r -v 4 {world}',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    return LaunchDescription([gazebo_resource_path, gazebo])
