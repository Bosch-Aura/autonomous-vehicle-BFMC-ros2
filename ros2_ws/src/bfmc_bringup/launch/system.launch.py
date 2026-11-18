"""Arranca todo el sistema AURA.

Cada nodo se lanza con el config/params.yaml de su propio paquete (más lo que haya
guardado con su botón Guardar, si tiene `persist_file`): acá no hay parámetros
duplicados. Las opciones solo activan o desactivan partes del sistema, eligen el
puerto serial y eligen los módulos intercambiables (ADR-0004): el detector de carril y el
algoritmo de giro. Sin elegirlos, corre lo que diga el params.yaml de cada paquete.

    ros2 launch bfmc_bringup system.launch.py
    ros2 launch bfmc_bringup system.launch.py camera:=false
    ros2 launch bfmc_bringup system.launch.py signs:=false serial_port:=/dev/ttyACM0
    ros2 launch bfmc_bringup system.launch.py lane_detector:=l_adaptativo \
        control_algorithm:=pure_pursuit
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


def _del_yaml(paquete, nodo, clave):
    """Un valor del params.yaml de un paquete, para usarlo de predeterminado sin repetirlo acá."""
    ruta = os.path.join(_compartida(paquete), 'config', 'params.yaml')
    with open(ruta, encoding='utf-8') as archivo:
        return str(yaml.safe_load(archivo)[nodo]['ros__parameters'][clave])


def generate_launch_description():
    camara = LaunchConfiguration('camera')
    senales = LaunchConfiguration('signs')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera', default_value='true',
            description='Arranca el driver de la cámara RealSense.'),
        DeclareLaunchArgument(
            'signs', default_value='true',
            description='Lanza el nodo de detección de señales. Arranca APAGADO: no carga el '
                        'modelo ni gasta CPU hasta que se lo prende (visor, o ros2 param set '
                        '/sign_detection enabled true). Para prenderlo hacen falta Ultralytics y '
                        'el modelo (ver el README de bfmc_sign_detection).'),
        DeclareLaunchArgument(
            'serial_port', default_value=_del_yaml(
                'bfmc_hardware_bridge', 'hardware_bridge', 'serial_port'),
            description='Puerto serial del ESP32.'),
        DeclareLaunchArgument(
            'lane_detector', default_value=_del_yaml(
                'bfmc_lane_detection', 'lane_detection', 'detector'),
            description='Detector de carril: hsv_ventanas o l_adaptativo (ADR-0004).'),
        DeclareLaunchArgument(
            'control_algorithm', default_value=_del_yaml('bfmc_control', 'control', 'algorithm'),
            description='Algoritmo de giro: pid o pure_pursuit (ADR-0004).'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(_compartida('bfmc_camera'), 'launch', 'camera.launch.py')),
            condition=IfCondition(camara)),

        _nodo('bfmc_lane_detection', 'lane_detection',
              [{'detector': LaunchConfiguration('lane_detector')}]),
        _nodo('bfmc_sign_detection', 'sign_detection', condicion=IfCondition(senales)),
        _nodo('bfmc_control', 'control',
              [{'algorithm': LaunchConfiguration('control_algorithm')}]),
        _nodo('bfmc_hardware_bridge', 'hardware_bridge',
              [{'serial_port': LaunchConfiguration('serial_port')}]),
        _nodo('bfmc_maneuvers', 'maneuvers'),
        _nodo('bfmc_decision', 'decision'),
        _nodo('bfmc_recorder', 'recorder'),
    ])
