import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('bfmc_camera'), 'config', 'params.yaml')
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            # Sin espacio de nombres los topics salen como /camera/color/image_raw.
            namespace='',
            parameters=[params],
            output='screen',
        ),
    ])
