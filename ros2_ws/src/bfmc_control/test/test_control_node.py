"""Tests del nodo de control: verifican la conexión entre mensajes y lógica.

Usan los parámetros reales de config/params.yaml, así el test falla si el YAML y
el código se desincronizan. Esperan mensajes reales, por eso tienen un tope de
tiempo; las reglas del PID se prueban aparte, sin ROS, en test_control_logic.py.
"""
from pathlib import Path
import time

from ackermann_msgs.msg import AckermannDriveStamped
from bfmc_interfaces.msg import LaneState
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
import yaml

from bfmc_control.control_node import ControlNode

PARAMS = Path(__file__).resolve().parents[1] / 'config' / 'params.yaml'
TIMEOUT_S = 5.0


@pytest.fixture(scope='module')
def parametros_del_yaml():
    with open(PARAMS, encoding='utf-8') as archivo:
        return yaml.safe_load(archivo)['control']['ros__parameters']


@pytest.fixture(scope='module')
def ros():
    # Cada paquete usa su propio dominio de ROS: colcon corre los tests de varios paquetes a la
    # vez y, en el mismo dominio, se cruzarían los mensajes de topics con el mismo nombre.
    rclpy.init(args=['--ros-args', '--params-file', str(PARAMS)], domain_id=11)
    yield
    rclpy.shutdown()


@pytest.fixture
def entorno(ros):
    nodo = ControlNode()
    auxiliar = rclpy.create_node('auxiliar_de_test_control')
    executor = SingleThreadedExecutor()
    executor.add_node(nodo)
    executor.add_node(auxiliar)
    yield nodo, auxiliar, executor
    executor.shutdown()
    nodo.destroy_node()
    auxiliar.destroy_node()


def _enviar_y_esperar_comando(entorno, estado):
    nodo, auxiliar, executor = entorno
    recibidos = []
    auxiliar.create_subscription(
        AckermannDriveStamped, nodo.get_parameter('cmd_topic').value,
        recibidos.append, 10)
    publicador = auxiliar.create_publisher(
        LaneState, nodo.get_parameter('lane_topic').value, 10)

    limite = time.monotonic() + TIMEOUT_S
    while not recibidos and time.monotonic() < limite:
        publicador.publish(estado)
        executor.spin_once(timeout_sec=0.1)
    return recibidos[0] if recibidos else None


def _estado(lateral, detectado=True):
    estado = LaneState()
    estado.detected = detectado
    estado.lateral_error_m = lateral
    estado.confidence = 1.0
    return estado


def test_el_nodo_se_llama_control(entorno):
    assert entorno[0].get_name() == 'control'


def test_los_valores_del_pid_salen_del_yaml(entorno, parametros_del_yaml):
    nodo = entorno[0]
    assert nodo.get_parameter('kp').value == parametros_del_yaml['kp']
    assert nodo.get_parameter('max_steer_rad').value == parametros_del_yaml['max_steer_rad']


def test_un_error_a_la_izquierda_publica_un_giro_a_la_izquierda(entorno):
    comando = _enviar_y_esperar_comando(entorno, _estado(lateral=0.2))
    assert comando is not None
    assert comando.drive.steering_angle > 0.0
    assert comando.drive.speed > 0.0


def test_un_carril_no_detectado_publica_direccion_al_centro_y_velocidad_reducida(
        entorno, parametros_del_yaml):
    comando = _enviar_y_esperar_comando(entorno, _estado(lateral=0.2, detectado=False))
    esperado = (parametros_del_yaml['cruise_speed_mps']
                * parametros_del_yaml['lost_speed_factor'])
    assert comando.drive.steering_angle == 0.0
    assert comando.drive.speed == pytest.approx(esperado)
