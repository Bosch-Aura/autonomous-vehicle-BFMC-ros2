import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('bfmc_decision'), 'config', 'params.yaml')
    return LaunchDescription([
        Node(
            package='bfmc_decision',
            executable='decision',
            name='decision',
            parameters=[params],
            output='screen',
        ),
    ])
