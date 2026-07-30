import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'turret_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='acer',
    maintainer_email='emreerenatalay66@gmail.com',
    description='Gorsel hizalama (taret PID) ve waypoint gorev akisi nodlari',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turret_aim_node = turret_mission.turret_aim_node:main',
            'mission_node = turret_mission.mission_node:main',
            'mock_target_publisher = turret_mission.mock_target_publisher:main',
            'simple_twist_mux = turret_mission.simple_twist_mux:main',
            'obstacle_avoidance_node = turret_mission.obstacle_avoidance_node:main',
            'vision_avoidance_node = turret_mission.vision_avoidance_node:main',
            'fused_avoidance_node = turret_mission.fused_avoidance_node:main',
        ],
    },
)
