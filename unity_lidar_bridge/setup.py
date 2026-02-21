from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'unity_lidar_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jayeshwar',
    maintainer_email='jayeshwar@todo.todo',
    description='Bridge between Unity LiDAR raycast and ROS2 LaserScan',
    license='MIT',
    entry_points={
        'console_scripts': [
            'lidar_bridge_node = unity_lidar_bridge.lidar_bridge_node:main',
        ],
    },
)
