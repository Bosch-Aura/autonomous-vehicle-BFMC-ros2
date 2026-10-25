import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('bfmc_sign_detection'), 'config', 'params.yaml')
    return LaunchDescription([
        Node(
            package='bfmc_sign_detection',
            executable='sign_detection',
            name='sign_detection',
            parameters=[params],
            output='screen',
        ),
    ])
