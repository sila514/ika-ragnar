"""Gorev 7 test bringup: turret_aim_node + sahte /detected_targets yayinlayici.

Kullanim:
  ros2 launch turret_mission turret_test.launch.py

Gercek Hailo/kamera yerine mock_target_publisher, /turret_cmd'yi dinleyip
hedefin ekrandaki piksel hatasini kapali cevrimde simule eder.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('turret_mission')
    turret_params = os.path.join(pkg_share, 'config', 'turret_params.yaml')

    return LaunchDescription([
        Node(
            package='turret_mission',
            executable='turret_aim_node',
            name='turret_aim_node',
            output='screen',
            parameters=[turret_params],
        ),
        Node(
            package='turret_mission',
            executable='mock_target_publisher',
            name='mock_target_publisher',
            output='screen',
            parameters=[{
                'image_width': 640,
                'image_height': 480,
                'mode': 'intermittent',
                'visible_duration': 6.0,
                'hidden_duration': 3.0,
                'initial_err_x': 180.0,
                'initial_err_y': -90.0,
            }],
        ),
    ])
