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
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
    ],

    install_requires=['cflib', 'setuptools'],
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
          'swarm_node = cf_control.swarm_node:main',
          'wand_node = cf_control.wand_node:main',
          'wand_driver_node = cf_control.wand_driver_node:main',
        ],
    },
)
