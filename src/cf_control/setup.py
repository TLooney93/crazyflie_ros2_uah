from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'cf_control'

setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(exclude=['test']),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),

        # Install Lighthouse YAML configuration files
        (
            os.path.join('share', package_name, 'config', 'lighthouse'),
            glob('config/lighthouse/*.yaml')
        ),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='tanner-looney',
    maintainer_email='tannerlooney@gmail.com',

    description='ROS 2 control package for Crazyflie swarm operations.',
    license='TODO: License declaration',

    extras_require={
        'test': [
            'pytest',
        ],
    },

    entry_points={
        'console_scripts': [
          'crazyflie_node = cf_control.crazyflie_node:main',
          'wand_node = cf_control.wand_node:main',
          'wand_driver_node = cf_control.wand_driver_node:main',
          'wand_coordinator = cf_control.wand_coordinator:main',
          'wand_button_node = cf_control.wand_button_node:main',
          'wand_gesture_node = cf_control.wand_gesture_node:main',
          'flight_wand_node = cf_control.flight_wand_node:main',
          'gesture_trial_logger = cf_control.gesture_trial_logger:main',
          'r2_command_test = cf_control.r2_command_test:main',
        ],
    },
)
