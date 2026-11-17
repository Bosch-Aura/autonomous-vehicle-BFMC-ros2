"""Lanza solo la detección de carril.

    ros2 launch bfmc_lane_detection lane_detection.launch.py
    ros2 launch bfmc_lane_detection lane_detection.launch.py detector:=l_adaptativo
"""
import os

from ament_index_python.packages import get_package_share_directory
from bfmc_lane_detection.param_store import archivo_persistido
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('bfmc_lane_detection'), 'config', 'params.yaml')
    with open(params, encoding='utf-8') as archivo:
        detector = yaml.safe_load(archivo)['lane_detection']['ros__parameters']['detector']
    # Lo guardado con el botón Guardar (si existe) va después: le gana al params.yaml. El
    # detector elegido al lanzar va al final: le gana a los dos.
    guardado = archivo_persistido(params, 'lane_detection')
    return LaunchDescription([
        DeclareLaunchArgument(
            'detector', default_value=detector,
            description='Detector de carril: hsv_ventanas o l_adaptativo (ADR-0004).'),
        Node(
            package='bfmc_lane_detection',
            executable='lane_detection',
            name='lane_detection',
            parameters=[params] + ([guardado] if guardado else [])
            + [{'detector': LaunchConfiguration('detector')}],
            output='screen',
        ),
    ])
