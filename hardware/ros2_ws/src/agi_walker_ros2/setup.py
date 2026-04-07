from setuptools import setup
import os
from glob import glob

package_name = 'agi_walker_ros2'

setup(
    name=package_name,
    version='4.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # URDF files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        # RViz configs
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AGI-Walker Team',
    maintainer_email='openneuro@example.com',
    description='ROS 2 integration for AGI-Walker',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_node = agi_walker_ros2.robot_node:main',
            'state_publisher = agi_walker_ros2.state_publisher:main',
            'cmd_relay = agi_walker_ros2.cmd_relay:main',
        ],
    },
)
