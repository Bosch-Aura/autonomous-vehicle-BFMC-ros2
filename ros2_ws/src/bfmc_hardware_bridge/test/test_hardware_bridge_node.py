"""Tests del nodo del puente con un puerto simulado: no necesitan el ESP32.

Usan los parámetros reales de config/params.yaml. Esperan mensajes reales, por
eso tienen un tope de tiempo; las reglas se prueban aparte, sin ROS, en
test_hardware_bridge_logic.py.
"""
import json
import math
from pathlib import Path
import time

from ackermann_msgs.msg import AckermannDriveStamped
from bfmc_hardware_bridge.hardware_bridge_node import HardwareBridgeNode
from bfmc_interfaces.msg import HardwareStatus, SafetyStatus
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Range
import yaml

PARAMS = Path(__file__).resolve().parents[1] / 'config' / 'params.yaml'
TIMEOUT_S = 5.0
RELIABLE = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
BEST_EFFORT = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)


class PuertoFalso:
    """Hace de ESP32: guarda lo que se le envía y entrega lo que se le programa."""

    def __init__(self):
        self.escritos = bytearray()
        self.por_leer = bytearray()
        self.cerrado = False
        self.fallar = False

    def write(self, datos):
        if self.fallar:
            raise OSError('cable desconectado')
        self.escritos += datos

    def read_available(self):
        if self.fallar:
            raise OSError('cable desconectado')
        datos = bytes(self.por_leer)
        self.por_leer.clear()
        return datos

    def close(self):
        self.cerrado = True

    def tramas(self):
        return [json.loads(linea) for linea in bytes(self.escritos).decode().splitlines()]


@pytest.fixture(scope='module')
def parametros_del_yaml():
    with open(PARAMS, encoding='utf-8') as archivo:
        return yaml.safe_load(archivo)['hardware_bridge']['ros__parameters']


@pytest.fixture(scope='module')
def ros():
    # Se acorta la espera de reconexión para no hacer lento el test de caída del puerto.
    # Cada paquete usa su propio dominio de ROS: colcon corre los tests de varios paquetes a la
    # vez y, en el mismo dominio, se cruzarían los mensajes de topics con el mismo nombre.
    rclpy.init(args=['--ros-args', '--params-file', str(PARAMS),
                     '-p', 'reconnect_period_s:=0.1'], domain_id=13)
    yield
    rclpy.shutdown()


class Entorno:

    def __init__(self, nodo, auxiliar, executor, puerto):
        self.nodo = nodo
        self.auxiliar = auxiliar
        self.executor = executor
        self.puerto = puerto
        self.rangos = []
        self.estados = []
        self.aperturas = 0
        param = nodo.get_parameter
        self.pub_cmd = auxiliar.create_publisher(
            AckermannDriveStamped, param('cmd_topic').value, RELIABLE)
        self.pub_seguridad = auxiliar.create_publisher(
            SafetyStatus, param('safety_topic').value, RELIABLE)
        auxiliar.create_subscription(
            Range, param('ultrasonic_topic').value, self.rangos.append, BEST_EFFORT)
        auxiliar.create_subscription(
            HardwareStatus, param('status_topic').value, self.estados.append, RELIABLE)

    def esperar(self, condicion, publicar=None):
        limite = time.monotonic() + TIMEOUT_S
        while time.monotonic() < limite:
            if publicar is not None:
                publicar()
            self.executor.spin_once(timeout_sec=0.05)
            if condicion():
                return True
        return False

    def comando(self, velocidad, direccion):
        mensaje = AckermannDriveStamped()
        mensaje.drive.speed = velocidad
        mensaje.drive.steering_angle = direccion
        self.pub_cmd.publish(mensaje)

    def emergencia(self, activa):
        mensaje = SafetyStatus()
        mensaje.emergency = activa
        self.pub_seguridad.publish(mensaje)


@pytest.fixture
def entorno(ros):
    puerto = PuertoFalso()
    aperturas = []

    def fabrica(ruta, baud):
        aperturas.append((ruta, baud))
        return puerto

    nodo = HardwareBridgeNode(port_factory=fabrica)
    auxiliar = rclpy.create_node('auxiliar_de_test_bridge')
    executor = SingleThreadedExecutor()
    executor.add_node(nodo)
    executor.add_node(auxiliar)
    contexto = Entorno(nodo, auxiliar, executor, puerto)
    contexto.aperturas = aperturas
    yield contexto
    executor.shutdown()
    nodo.destroy_node()
    auxiliar.destroy_node()


def test_el_nodo_abre_el_puerto_y_la_velocidad_con_los_valores_del_yaml(
        entorno, parametros_del_yaml):
    assert entorno.esperar(lambda: entorno.aperturas)
    assert entorno.aperturas[0] == (parametros_del_yaml['serial_port'],
                                    parametros_del_yaml['baud_rate'])


def test_sin_comandos_el_nodo_envia_freno_como_heartbeat(entorno):
    assert entorno.esperar(lambda: len(entorno.puerto.tramas()) >= 3)
    tramas = entorno.puerto.tramas()
    assert all(trama['t'] == 'brake' for trama in tramas)
    assert [t['seq'] for t in tramas[:3]] == [0, 1, 2]


def test_un_comando_se_envia_al_esp32_en_sus_unidades(entorno):
    def hay_comando():
        return any(t['t'] == 'drive' for t in entorno.puerto.tramas())

    assert entorno.esperar(hay_comando, lambda: entorno.comando(0.25, math.radians(10.0)))
    trama = next(t for t in entorno.puerto.tramas() if t['t'] == 'drive')
    assert trama['v'] == 250
    assert trama['s'] == 100


def test_una_emergencia_envia_freno_aunque_siga_llegando_un_comando(entorno):
    def publicar():
        entorno.comando(0.25, 0.0)
        entorno.emergencia(True)

    def ultima_es_freno():
        tramas = entorno.puerto.tramas()
        return len(tramas) > 10 and all(t['t'] == 'brake' for t in tramas[-5:])

    assert entorno.esperar(ultima_es_freno, publicar)


def test_una_lectura_valida_del_esp32_se_publica_como_range(entorno):
    entorno.puerto.por_leer += b'{"t":"ultra","id":"front","dist_cm":87,"valid":true}\n'
    assert entorno.esperar(lambda: entorno.rangos)
    assert entorno.rangos[0].range == pytest.approx(0.87, abs=1e-5)
    assert entorno.rangos[0].radiation_type == Range.ULTRASOUND


def test_una_lectura_sin_eco_se_publica_como_camino_despejado(entorno):
    entorno.puerto.por_leer += b'{"t":"ultra","id":"front","dist_cm":0,"valid":false}\n'
    assert entorno.esperar(lambda: entorno.rangos)
    assert math.isinf(entorno.rangos[0].range) and entorno.rangos[0].range > 0


def test_el_estado_del_esp32_se_publica_como_hardware_status(entorno):
    entorno.puerto.por_leer += b'{"t":"status","fw":"0.1.0","wd":true,"last_seq":7}\n'
    assert entorno.esperar(
        lambda: any(e.link_up and e.firmware_version == '0.1.0' for e in entorno.estados))
    estado = next(e for e in entorno.estados if e.link_up)
    assert estado.watchdog_tripped is True
    assert estado.last_seq == 7


def test_sin_mensajes_del_esp32_el_enlace_se_publica_caido(entorno):
    assert entorno.esperar(lambda: entorno.estados)
    assert entorno.estados[0].link_up is False


def test_una_linea_de_ruido_no_levanta_el_enlace(entorno):
    entorno.puerto.por_leer += b'ruido\n'
    assert entorno.esperar(lambda: len(entorno.estados) >= 2)
    assert all(e.link_up is False for e in entorno.estados)


def test_si_el_puerto_falla_el_nodo_lo_cierra_y_lo_vuelve_a_abrir(entorno):
    assert entorno.esperar(lambda: len(entorno.aperturas) == 1)
    entorno.puerto.fallar = True
    assert entorno.esperar(lambda: entorno.puerto.cerrado)
    entorno.puerto.fallar = False
    assert entorno.esperar(lambda: len(entorno.aperturas) >= 2)
