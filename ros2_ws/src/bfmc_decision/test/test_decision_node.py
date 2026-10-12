"""Tests del nodo de decisión: verifican la conexión entre topics, acción y lógica.

Un servidor de acción falso hace de bfmc_maneuvers. Usan los parámetros reales de
config/params.yaml. Esperan mensajes reales, por eso tienen un tope de tiempo;
las reglas de decisión se prueban aparte, sin ROS, en test_decision_logic.py.
"""
import gc
from pathlib import Path
import time

from ackermann_msgs.msg import AckermannDriveStamped
from bfmc_interfaces.action import ManeuverExecute
from bfmc_interfaces.msg import (
    HardwareStatus, LaneState, SafetyStatus, SignDetection, SignDetectionArray, VehicleState,
)
import pytest
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy

from bfmc_decision.decision_logic import State
from bfmc_decision.decision_node import DecisionNode

PARAMS = Path(__file__).resolve().parents[1] / 'config' / 'params.yaml'
TIMEOUT_S = 10.0
RELIABLE = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
CARRIL = (0.25, 0.125)
MANIOBRA = (0.5, -0.25)


@pytest.fixture(scope='module')
def ros():
    # Cada paquete usa su propio dominio de ROS: colcon corre los tests de varios paquetes a la
    # vez y, en el mismo dominio, se cruzarían los mensajes de topics con el mismo nombre.
    rclpy.init(args=['--ros-args', '--params-file', str(PARAMS)], domain_id=15)
    yield
    # Se liberan los objetos de rclpy antes de cerrarlo: si el recolector los
    # liberara después, el intérprete podría terminar con un fallo de segmentación.
    gc.collect()
    rclpy.shutdown()


class Entorno:
    """El nodo bajo prueba, un servidor de maniobras falso y los publicadores de entrada."""

    def __init__(self, nodo, auxiliar, executor, con_servidor):
        self.nodo = nodo
        self.executor = executor
        self._servidor = None
        self._cerrado = False
        self.comandos = []
        self.estados = []
        self.metas = []
        param = nodo.get_parameter

        crear = auxiliar.create_publisher
        self.pub_lane = crear(LaneState, param('lane_topic').value, RELIABLE)
        self.pub_signs = crear(SignDetectionArray, param('signs_topic').value, RELIABLE)
        self.pub_safety = crear(SafetyStatus, param('safety_topic').value, RELIABLE)
        self.pub_hardware = crear(HardwareStatus, param('hardware_topic').value, RELIABLE)
        self.pub_lane_cmd = crear(
            AckermannDriveStamped, param('lane_cmd_topic').value, RELIABLE)
        self.pub_maneuver_cmd = crear(
            AckermannDriveStamped, param('maneuver_cmd_topic').value, RELIABLE)
        auxiliar.create_subscription(
            AckermannDriveStamped, param('drive_topic').value, self.comandos.append, RELIABLE)
        auxiliar.create_subscription(
            VehicleState, param('state_topic').value, self.estados.append, RELIABLE)

        if con_servidor:
            self._servidor = ActionServer(
                auxiliar, ManeuverExecute, param('maneuver_action_name').value,
                execute_callback=self._ejecutar, callback_group=ReentrantCallbackGroup())

    def _ejecutar(self, goal_handle):
        """Falso bfmc_maneuvers: publica el comando de maniobra durante 0.4 s y termina."""
        self.metas.append((goal_handle.request.maneuver, goal_handle.request.direction))
        fin = time.monotonic() + 0.4
        while time.monotonic() < fin and not self._cerrado:
            self.pub_maneuver_cmd.publish(_comando(*MANIOBRA))
            time.sleep(0.05)
        goal_handle.succeed()
        resultado = ManeuverExecute.Result()
        resultado.success = True
        return resultado

    def publicar_entradas(self, emergencia=False, senal=None):
        """Publica un conjunto de entradas sanas; cada llamada las deja frescas."""
        carril = LaneState()
        carril.detected = True
        carril.confidence = 1.0
        self.pub_lane.publish(carril)

        seguridad = SafetyStatus()
        seguridad.emergency = emergencia
        seguridad.reason = (SafetyStatus.REASON_OBSTACLE if emergencia
                            else SafetyStatus.REASON_NONE)
        seguridad.sensor_valid = True
        self.pub_safety.publish(seguridad)

        hardware = HardwareStatus()
        hardware.link_up = True
        self.pub_hardware.publish(hardware)

        self.pub_lane_cmd.publish(_comando(*CARRIL))

        if senal is not None:
            deteccion = SignDetection()
            deteccion.label = senal
            deteccion.confidence = 0.9
            deteccion.distance_m = 0.5
            senales = SignDetectionArray()
            senales.detections = [deteccion]
            self.pub_signs.publish(senales)

    def destruir(self):
        self._cerrado = True
        if self._servidor is not None:
            self._servidor.destroy()

    def esperar(self, condicion, **entradas):
        limite = time.monotonic() + TIMEOUT_S
        while time.monotonic() < limite:
            self.publicar_entradas(**entradas)
            self.executor.spin_once(timeout_sec=0.05)
            if condicion():
                return True
        return False

    def ultimo_estado(self):
        return self.estados[-1].state if self.estados else None

    def ultimo_comando(self):
        if not self.comandos:
            return None
        drive = self.comandos[-1].drive
        return (drive.speed, drive.steering_angle)


def _comando(velocidad, direccion):
    mensaje = AckermannDriveStamped()
    mensaje.drive.speed = velocidad
    mensaje.drive.steering_angle = direccion
    return mensaje


def _armar(ros, con_servidor):
    nodo = DecisionNode()
    auxiliar = rclpy.create_node('auxiliar_de_test_decision')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(nodo)
    executor.add_node(auxiliar)
    return nodo, auxiliar, executor, Entorno(nodo, auxiliar, executor, con_servidor)


@pytest.fixture
def entorno(ros):
    nodo, auxiliar, executor, contexto = _armar(ros, con_servidor=True)
    yield contexto
    executor.shutdown()
    contexto.destruir()
    nodo.destroy_node()
    auxiliar.destroy_node()


@pytest.fixture
def entorno_sin_servidor(ros):
    nodo, auxiliar, executor, contexto = _armar(ros, con_servidor=False)
    yield contexto
    executor.shutdown()
    contexto.destruir()
    nodo.destroy_node()
    auxiliar.destroy_node()


def test_los_estados_de_la_logica_coinciden_con_los_del_mensaje():
    assert int(State.STARTING) == VehicleState.STATE_STARTING
    assert int(State.LANE_FOLLOW) == VehicleState.STATE_LANE_FOLLOW
    assert int(State.MANEUVER) == VehicleState.STATE_MANEUVER
    assert int(State.EMERGENCY) == VehicleState.STATE_EMERGENCY


def test_el_nodo_es_el_unico_publicador_del_comando_final(entorno):
    drive_topic = entorno.nodo.get_parameter('drive_topic').value
    assert entorno.nodo.count_publishers(drive_topic) == 1


def test_sin_entradas_el_nodo_publica_arranque_y_comando_cero(entorno):
    limite = time.monotonic() + TIMEOUT_S
    while not entorno.estados and time.monotonic() < limite:
        entorno.executor.spin_once(timeout_sec=0.05)
    assert entorno.estados[0].state == VehicleState.STATE_STARTING
    assert entorno.ultimo_comando() == (0.0, 0.0)


def test_con_las_entradas_sanas_reenvia_el_comando_del_carril(entorno):
    assert entorno.esperar(lambda: entorno.ultimo_comando() == pytest.approx(CARRIL))
    assert entorno.ultimo_estado() == VehicleState.STATE_LANE_FOLLOW


def test_una_emergencia_de_seguridad_detiene_el_auto_y_se_informa(entorno):
    assert entorno.esperar(
        lambda: entorno.ultimo_estado() == VehicleState.STATE_EMERGENCY, emergencia=True)
    assert entorno.ultimo_comando() == (0.0, 0.0)
    assert 'obstáculo' in entorno.estados[-1].reason


def test_un_stop_cercano_pide_la_maniobra_y_usa_su_comando_mientras_dura(entorno):
    assert entorno.esperar(lambda: entorno.metas, senal='stop')
    assert entorno.metas[0] == ('stop', '')

    assert entorno.esperar(
        lambda: entorno.ultimo_comando() == pytest.approx(MANIOBRA), senal='stop')
    assert entorno.ultimo_estado() == VehicleState.STATE_MANEUVER


def test_al_terminar_la_maniobra_se_vuelve_a_seguir_el_carril(entorno):
    assert entorno.esperar(lambda: entorno.metas, senal='stop')
    assert entorno.esperar(
        lambda: entorno.ultimo_estado() == VehicleState.STATE_MANEUVER, senal='stop')
    assert entorno.esperar(
        lambda: entorno.ultimo_estado() == VehicleState.STATE_LANE_FOLLOW, senal='stop')
    assert len(entorno.metas) == 1


def test_sin_servidor_de_maniobras_el_pedido_se_rechaza_y_el_auto_sigue_el_carril(
        entorno_sin_servidor):
    assert entorno_sin_servidor.esperar(
        lambda: entorno_sin_servidor.ultimo_comando() == pytest.approx(CARRIL), senal='stop')
    assert entorno_sin_servidor.esperar(
        lambda: entorno_sin_servidor.ultimo_estado() == VehicleState.STATE_LANE_FOLLOW,
        senal='stop')
