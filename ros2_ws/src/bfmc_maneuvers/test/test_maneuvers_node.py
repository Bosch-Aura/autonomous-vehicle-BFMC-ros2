"""Tests del action server de maniobras: verifican la conexión entre acción, estrategias y topic.

Usan los parámetros reales de config/params.yaml, con los tiempos acortados para
que el test no dure segundos. Las secuencias de cada maniobra se prueban aparte,
sin ROS, en test_stop_strategy.py y test_intersection_strategy.py.
"""
import gc
from pathlib import Path
import time

from ackermann_msgs.msg import AckermannDriveStamped
from action_msgs.msg import GoalStatus
from bfmc_interfaces.action import ManeuverExecute
from bfmc_maneuvers.maneuvers_node import ManeuversNode
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy

PARAMS = Path(__file__).resolve().parents[1] / 'config' / 'params.yaml'
TIMEOUT_S = 10.0

# Tiempos acortados: el test no debería durar segundos por maniobra.
ACORTAR = [
    'stop.hold_s:=0.3', 'stop.exit_s:=0.0',
    'intersection.left.approach_s:=0.2', 'intersection.left.turn_s:=0.3',
]


@pytest.fixture(scope='module')
def ros():
    argumentos = ['--ros-args', '--params-file', str(PARAMS)]
    for ajuste in ACORTAR:
        argumentos += ['-p', ajuste]
    # Cada paquete usa su propio dominio de ROS: colcon corre los tests de varios paquetes a la
    # vez y, en el mismo dominio, se cruzarían los mensajes de topics con el mismo nombre.
    rclpy.init(args=argumentos, domain_id=14)
    yield
    # Se liberan los objetos de rclpy antes de cerrarlo: si el recolector los
    # liberara después, el intérprete podría terminar con un fallo de segmentación.
    gc.collect()
    rclpy.shutdown()


class Entorno:

    def __init__(self, nodo, auxiliar, executor):
        self.nodo = nodo
        self.executor = executor
        self.comandos = []
        self.realimentaciones = []
        auxiliar.create_subscription(
            AckermannDriveStamped, nodo.get_parameter('cmd_topic').value,
            self.comandos.append, QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE))
        nombre_accion = nodo.get_parameter('action_name').value
        self.cliente = ActionClient(auxiliar, ManeuverExecute, nombre_accion)
        assert self.cliente.wait_for_server(timeout_sec=TIMEOUT_S)

    def esperar(self, futuro):
        self.executor.spin_until_future_complete(futuro, timeout_sec=TIMEOUT_S)
        assert futuro.done(), 'el futuro no se completó a tiempo'
        return futuro.result()

    def pedir(self, maniobra, direccion=''):
        meta = ManeuverExecute.Goal()
        meta.maneuver = maniobra
        meta.direction = direccion
        return self.esperar(self.cliente.send_goal_async(
            meta, feedback_callback=self.realimentaciones.append))

    def resultado(self, goal_handle):
        return self.esperar(goal_handle.get_result_async())


@pytest.fixture
def entorno(ros):
    nodo = ManeuversNode()
    auxiliar = rclpy.create_node('auxiliar_de_test_maneuvers')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(nodo)
    executor.add_node(auxiliar)
    contexto = Entorno(nodo, auxiliar, executor)
    yield contexto
    executor.shutdown()
    contexto.cliente.destroy()
    nodo.destroy_node()
    auxiliar.destroy_node()


def test_un_stop_se_acepta_se_completa_y_termina_con_exito(entorno):
    meta = entorno.pedir('stop')
    assert meta.accepted
    resultado = entorno.resultado(meta)
    assert resultado.status == GoalStatus.STATUS_SUCCEEDED
    assert resultado.result.success is True


def test_durante_un_stop_se_publica_un_comando_con_el_auto_detenido(entorno):
    entorno.resultado(entorno.pedir('stop'))
    durante = [c for c in entorno.comandos if c.drive.speed == 0.0]
    assert len(durante) >= 2
    assert all(c.drive.steering_angle == 0.0 for c in durante)


def test_se_informa_el_progreso_mientras_la_maniobra_esta_en_curso(entorno):
    entorno.resultado(entorno.pedir('stop'))
    tiempos = [r.feedback.elapsed_s for r in entorno.realimentaciones]
    assert len(tiempos) >= 2
    assert tiempos == sorted(tiempos)


def test_una_interseccion_a_la_izquierda_gira_con_direccion_positiva(entorno):
    resultado = entorno.resultado(entorno.pedir('intersection', 'left'))
    assert resultado.result.success is True
    assert any(c.drive.steering_angle > 0.0 and c.drive.speed > 0.0 for c in entorno.comandos)


def test_al_terminar_se_publica_un_comando_de_parada(entorno):
    entorno.resultado(entorno.pedir('intersection', 'left'))
    final = entorno.comandos[-1]
    assert (final.drive.speed, final.drive.steering_angle) == (0.0, 0.0)


@pytest.mark.parametrize('maniobra, direccion', [
    ('estacionar', ''),
    ('intersection', ''),
    ('intersection', 'atras'),
])
def test_un_pedido_invalido_se_rechaza(entorno, maniobra, direccion):
    assert not entorno.pedir(maniobra, direccion).accepted


def test_un_segundo_pedido_se_rechaza_mientras_hay_una_maniobra_activa(entorno):
    primera = entorno.pedir('stop')
    assert primera.accepted
    assert not entorno.pedir('stop').accepted
    entorno.resultado(primera)


def test_se_puede_pedir_otra_maniobra_cuando_la_anterior_termino(entorno):
    entorno.resultado(entorno.pedir('stop'))
    segunda = entorno.pedir('stop')
    assert segunda.accepted
    entorno.resultado(segunda)


def test_una_maniobra_se_puede_cancelar_y_deja_el_auto_detenido(entorno):
    meta = entorno.pedir('intersection', 'left')
    assert meta.accepted
    time.sleep(0.1)
    entorno.esperar(meta.cancel_goal_async())
    resultado = entorno.resultado(meta)
    assert resultado.status == GoalStatus.STATUS_CANCELED
    final = entorno.comandos[-1]
    assert (final.drive.speed, final.drive.steering_angle) == (0.0, 0.0)
