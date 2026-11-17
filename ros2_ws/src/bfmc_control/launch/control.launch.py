"""Lanza solo el control.

    ros2 launch bfmc_control control.launch.py
    ros2 launch bfmc_control control.launch.py algorithm:=pure_pursuit
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('bfmc_control'), 'config', 'params.yaml')
    with open(params, encoding='utf-8') as archivo:
        algoritmo = yaml.safe_load(archivo)['control']['ros__parameters']['algorithm']
    return LaunchDescription([
        DeclareLaunchArgument(
            'algorithm', default_value=algoritmo,
            description='Algoritmo de giro: pid o pure_pursuit (ADR-0004).'),
        Node(
            package='bfmc_control',
            executable='control',
            name='control',
            parameters=[params, {'algorithm': LaunchConfiguration('algorithm')}],
            output='screen',
        ),
    ])
