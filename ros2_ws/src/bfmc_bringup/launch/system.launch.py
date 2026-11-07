"""Arranca todo el sistema AURA.

Cada nodo se lanza con el config/params.yaml de su propio paquete (más lo que haya
guardado con su botón Guardar, si tiene `persist_file`): acá no hay parámetros
duplicados. Las opciones solo activan o desactivan partes del sistema y eligen el
puerto serial.

    ros2 launch bfmc_bringup system.launch.py
    ros2 launch bfmc_bringup system.launch.py camera:=false
    ros2 launch bfmc_bringup system.launch.py signs:=true serial_port:=/dev/ttyACM0
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _compartida(paquete):
    return get_package_share_directory(paquete)


def _guardado(params_yaml, nodo):
    """El archivo que guardó el nodo con su botón Guardar (`persist_file`), si existe."""
    with open(params_yaml, encoding='utf-8') as archivo:
        ruta = yaml.safe_load(archivo)[nodo]['ros__parameters'].get('persist_file')
    ruta = os.path.expanduser(ruta) if ruta else None
    return [ruta] if ruta and os.path.exists(ruta) else []


def _nodo(paquete, ejecutable, parametros_extra=None, condicion=None):
    """Lanza un nodo con el params.yaml de su paquete. El nombre del nodo es el del ejecutable.

    Si el nodo guardó parámetros (`persist_file`), ese archivo va después: le gana al YAML.
    """
    params_yaml = os.path.join(_compartida(paquete), 'config', 'params.yaml')
    parametros = [params_yaml] + _guardado(params_yaml, ejecutable)
    parametros += parametros_extra or []
    return Node(package=paquete, executable=ejecutable, name=ejecutable,
                parameters=parametros, output='screen', condition=condicion)


def _puerto_por_defecto():
    """El puerto serial que trae el YAML del puente, para no repetirlo acá."""
    ruta = os.path.join(_compartida('bfmc_hardware_bridge'), 'config', 'params.yaml')
    with open(ruta, encoding='utf-8') as archivo:
        return yaml.safe_load(archivo)['hardware_bridge']['ros__parameters']['serial_port']


def generate_launch_description():
    camara = LaunchConfiguration('camera')
    senales = LaunchConfiguration('signs')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera', default_value='true',
            description='Arranca el driver de la cámara RealSense.'),
        DeclareLaunchArgument(
            'signs', default_value='false',
            description='Arranca la detección de señales. Necesita Ultralytics y el modelo '
                        '(ver el README de bfmc_sign_detection).'),
        DeclareLaunchArgument(
            'serial_port', default_value=_puerto_por_defecto(),
            description='Puerto serial del ESP32.'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(_compartida('bfmc_camera'), 'launch', 'camera.launch.py')),
            condition=IfCondition(camara)),

        _nodo('bfmc_lane_detection', 'lane_detection'),
        _nodo('bfmc_sign_detection', 'sign_detection', condicion=IfCondition(senales)),
        _nodo('bfmc_control', 'control'),
        _nodo('bfmc_hardware_bridge', 'hardware_bridge',
              [{'serial_port': LaunchConfiguration('serial_port')}]),
        _nodo('bfmc_maneuvers', 'maneuvers'),
        _nodo('bfmc_decision', 'decision'),
    ])
