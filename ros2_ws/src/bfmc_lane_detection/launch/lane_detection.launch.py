import os

from ament_index_python.packages import get_package_share_directory
from bfmc_lane_detection.param_store import archivo_persistido
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('bfmc_lane_detection'), 'config', 'params.yaml')
    # Lo guardado con el botón Guardar (si existe) va después: le gana al params.yaml.
    guardado = archivo_persistido(params, 'lane_detection')
    return LaunchDescription([
        Node(
            package='bfmc_lane_detection',
            executable='lane_detection',
            name='lane_detection',
            parameters=[params] + ([guardado] if guardado else []),
            output='screen',
        ),
    ])
