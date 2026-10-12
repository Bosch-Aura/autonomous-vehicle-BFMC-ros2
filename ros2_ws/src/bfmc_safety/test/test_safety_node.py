"""Tests del nodo de seguridad: verifican la conexión entre mensajes y lógica.

Usan los parámetros reales de config/params.yaml. Esperan mensajes reales, por
eso tienen un tope de tiempo; las reglas de seguridad se prueban aparte, sin
ROS, en test_safety_logic.py.
"""
import math
from pathlib import Path
import time

from bfmc_interfaces.msg import SafetyStatus
from bfmc_safety.safety_logic import Reason
from bfmc_safety.safety_node import SafetyNode
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Range

PARAMS = Path(__file__).resolve().parents[1] / 'config' / 'params.yaml'
TIMEOUT_S = 5.0


@pytest.fixture(scope='module')
def ros():
    # Cada paquete usa su propio dominio de ROS: colcon corre los tests de varios paquetes a la
    # vez y, en el mismo dominio, se cruzarían los mensajes de topics con el mismo nombre.
    rclpy.init(args=['--ros-args', '--params-file', str(PARAMS)], domain_id=12)
    yield
    rclpy.shutdown()


@pytest.fixture
def entorno(ros):
    nodo = SafetyNode()
    auxiliar = rclpy.create_node('auxiliar_de_test_safety')
    estados = []
    auxiliar.create_subscription(
        SafetyStatus, nodo.get_parameter('status_topic').value, estados.append,
        QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
    publicador = auxiliar.create_publisher(
        Range, nodo.get_parameter('ultrasonic_topic').value,
        QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
    executor = SingleThreadedExecutor()
    executor.add_node(nodo)
    executor.add_node(auxiliar)
    yield estados, publicador, executor
    executor.shutdown()
    nodo.destroy_node()
    auxiliar.destroy_node()


def _lectura(distancia):
    lectura = Range()
    lectura.radiation_type = Range.ULTRASOUND
    lectura.min_range = 0.02
    lectura.max_range = 4.0
    lectura.range = distancia
    return lectura


def _esperar(entorno, condicion, distancia=None):
    """Gira el executor, publicando `distancia` si se indica, hasta que se cumpla la condición."""
    estados, publicador, executor = entorno
    limite = time.monotonic() + TIMEOUT_S
    while time.monotonic() < limite:
        if distancia is not None:
            publicador.publish(_lectura(distancia))
        executor.spin_once(timeout_sec=0.05)
        if estados and condicion(estados[-1]):
            return estados[-1]
    return None


def test_los_motivos_coinciden_con_los_del_mensaje():
    assert int(Reason.NONE) == SafetyStatus.REASON_NONE
    assert int(Reason.OBSTACLE) == SafetyStatus.REASON_OBSTACLE
    assert int(Reason.SENSOR_INVALID) == SafetyStatus.REASON_SENSOR_INVALID
    assert int(Reason.SENSOR_STALE) == SafetyStatus.REASON_SENSOR_STALE


def test_sin_lecturas_se_publica_emergencia_por_sensor_vencido(entorno):
    estado = _esperar(entorno, lambda e: True)
    assert estado is not None
    assert estado.emergency is True
    assert estado.reason == SafetyStatus.REASON_SENSOR_STALE


def test_un_obstaculo_cercano_publica_emergencia_por_obstaculo(entorno):
    estado = _esperar(entorno, lambda e: e.reason == SafetyStatus.REASON_OBSTACLE, distancia=0.1)
    assert estado is not None
    assert estado.emergency is True
    assert estado.distance_m == pytest.approx(0.1, abs=1e-5)


def test_el_camino_despejado_publica_sin_emergencia(entorno):
    estado = _esperar(entorno, lambda e: not e.emergency, distancia=math.inf)
    assert estado is not None
    assert estado.reason == SafetyStatus.REASON_NONE
    assert estado.sensor_valid is True
