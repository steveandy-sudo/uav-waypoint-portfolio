from setuptools import setup
from glob import glob
import os

package_name = 'uav_waypoint_mission'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name,
            ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='junghun',
    maintainer_email='junghun@example.com',
    description='Simple UAV waypoint mission node for PX4 SITL and Gazebo Sim.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uav_waypoint_node = uav_waypoint_mission.uav_waypoint_node:main',
        ],
    },
)
