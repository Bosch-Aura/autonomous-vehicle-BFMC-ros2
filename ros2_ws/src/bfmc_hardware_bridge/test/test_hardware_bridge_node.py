"""Tests del nodo del puente con un puerto simulado: no necesitan el ESP32.

Usan los parámetros reales de config/params.yaml. Esperan mensajes reales, por
eso tienen un tope de tiempo; las reglas se prueban aparte, sin ROS, en
test_hardware_bridge_logic.py.
"""
import math
from pathlib import Path
import time

from ackermann_msgs.msg import AckermannDriveStamped
from bfmc_hardware_bridge.hardware_bridge_node import HardwareBridgeNode
from bfmc_interfaces.msg import HardwareStatus
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool, Trigger
import yaml

PARAMS = Path(__file__).resolve().parents[1] / 'config' / 'params.yaml'
TIMEOUT_S = 5.0
RELIABLE = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)


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
        return bytes(self.escritos).decode('ascii').splitlines()


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
        self.estados = []
        self.aperturas = 0
        param = nodo.get_parameter
        self.pub_cmd = auxiliar.create_publisher(
            AckermannDriveStamped, param('cmd_topic').value, RELIABLE)
        auxiliar.create_subscription(
            HardwareStatus, param('status_topic').value, self.estados.append, RELIABLE)
        self.cliente_armado = auxiliar.create_client(Trigger, param('arm_service').value)
        self.cliente_stop = auxiliar.create_client(SetBool, param('stop_service').value)

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


def test_al_abrir_el_puerto_arma_el_esp32_sin_pedir_modo(entorno):
    assert entorno.esperar(lambda: 'M:SYS_ARM:0' in entorno.puerto.tramas())
    assert not any('SYS_MODE' in trama for trama in entorno.puerto.tramas())


def test_el_auto_arranca_detenido_y_el_arranque_lo_suelta(entorno):
    assert entorno.esperar(lambda: 'M:SYS_STOP:0' in entorno.puerto.tramas())
    assert 'M:SYS_RESUME:0' not in entorno.puerto.tramas()


def test_sin_comandos_envia_velocidad_cero_como_heartbeat(entorno):
    assert entorno.esperar(lambda: entorno.puerto.tramas().count('C:SET_SPEED:0') >= 3)
    assert 'C:SET_STEER:105' in entorno.puerto.tramas()


def test_un_comando_se_envia_al_esp32_en_sus_unidades(entorno):
    assert entorno.esperar(lambda: 'C:SET_SPEED:255' in entorno.puerto.tramas(),
                           lambda: entorno.comando(0.5, math.radians(25.0)))
    tramas = entorno.puerto.tramas()
    assert 'C:SET_DIR:1' in tramas
    assert 'C:SET_STEER:50' in tramas


def test_el_estado_del_esp32_se_publica_tal_como_lo_reporta(entorno):
    def avisar():
        entorno.puerto.por_leer += b'EVENT:STATE_AUTO_TRANSITION:RUNNING->STOPPED:SYS_STOP\n'

    assert entorno.esperar(
        lambda: any(e.esp32_state == 'STOPPED' for e in entorno.estados), avisar)
    assert [e for e in entorno.estados if e.esp32_state == 'STOPPED'][-1].obstacle_stop is False


def test_un_obstaculo_del_esp32_se_publica_con_su_distancia_y_se_despeja(entorno):
    def avisar():
        entorno.puerto.por_leer += (
            b'EVENT:OBSTACLE_DETECTED:22\nEVENT:STATE_AUTO_TRANSITION:RUNNING->STOPPED:OBSTACLE\n')

    assert entorno.esperar(
        lambda: any(e.obstacle_stop for e in entorno.estados), avisar)
    detectado = [e for e in entorno.estados if e.obstacle_stop][-1]
    assert detectado.obstacle_distance_m == pytest.approx(0.22)
    assert detectado.watchdog_tripped is False

    def despejar():
        entorno.puerto.por_leer += (
            b'EVENT:OBSTACLE_CLEARED\nEVENT:STATE_AUTO_TRANSITION:STOPPED->RUNNING\n')

    entorno.estados.clear()
    assert entorno.esperar(
        lambda: entorno.estados and not entorno.estados[-1].obstacle_stop, despejar)
    assert math.isnan(entorno.estados[-1].obstacle_distance_m)


def test_un_obstaculo_no_hace_que_el_puente_mande_un_freno(entorno):
    def publicar():
        entorno.comando(0.5, 0.0)
        entorno.puerto.por_leer += b'EVENT:OBSTACLE_DETECTED:22\n'

    assert entorno.esperar(lambda: any(e.obstacle_stop for e in entorno.estados), publicar)
    assert 'E:BRAKE_NOW:0' not in entorno.puerto.tramas()
    assert 'C:SET_SPEED:255' in entorno.puerto.tramas()


def test_un_evento_del_esp32_levanta_el_enlace(entorno):
    def responder():
        entorno.puerto.por_leer += b'EVENT:CMD_RECEIVED:SET_SPEED:0\n'

    assert entorno.esperar(lambda: any(e.link_up for e in entorno.estados), responder)


def test_un_fault_del_esp32_se_publica_como_watchdog_tripped(entorno):
    def responder():
        entorno.puerto.por_leer += b'EVENT:WATCHDOG_TIMEOUT\nEVENT:STATE_CHANGED:FAULT\n'

    assert entorno.esperar(
        lambda: any(e.link_up and e.watchdog_tripped for e in entorno.estados), responder)


def test_el_servicio_de_armado_rearma_despues_de_un_fault(entorno):
    entorno.puerto.por_leer += b'EVENT:STATE_CHANGED:RUNNING\n'
    assert entorno.esperar(lambda: entorno.puerto.tramas().count('C:SET_SPEED:0') >= 2)
    entorno.puerto.por_leer += b'EVENT:STATE_CHANGED:FAULT\n'
    assert entorno.esperar(lambda: entorno.puerto.tramas().count('C:SET_SPEED:0') >= 4)
    assert 'M:SYS_ARM:0' not in entorno.puerto.tramas()

    assert entorno.cliente_armado.wait_for_service(timeout_sec=TIMEOUT_S)
    respuesta = entorno.cliente_armado.call_async(Trigger.Request())
    assert entorno.esperar(respuesta.done)
    assert respuesta.result().success is True
    assert entorno.esperar(lambda: 'M:SYS_ARM:0' in entorno.puerto.tramas())


def test_el_servicio_de_stop_detiene_y_arranca_el_esp32(entorno):
    assert entorno.cliente_stop.wait_for_service(timeout_sec=TIMEOUT_S)

    def pedir(detener):
        pedido = SetBool.Request()
        pedido.data = detener
        respuesta = entorno.cliente_stop.call_async(pedido)
        assert entorno.esperar(respuesta.done)
        assert respuesta.result().success is True

    pedir(True)
    assert entorno.esperar(lambda: 'M:SYS_STOP:0' in entorno.puerto.tramas())
    entorno.puerto.por_leer += b'EVENT:CMD_RECEIVED:SYS_STOP\n'
    pedir(False)
    assert entorno.esperar(lambda: 'M:SYS_RESUME:0' in entorno.puerto.tramas())


def test_sin_mensajes_del_esp32_el_enlace_se_publica_caido(entorno):
    assert entorno.esperar(lambda: entorno.estados)
    assert entorno.estados[0].link_up is False


def test_la_depuracion_del_esp32_no_levanta_el_enlace(entorno):
    entorno.puerto.por_leer += b'[MotorTask] Motor task started\n'
    assert entorno.esperar(lambda: len(entorno.estados) >= 2)
    assert all(e.link_up is False for e in entorno.estados)


def test_si_el_puerto_falla_el_nodo_lo_cierra_y_lo_vuelve_a_abrir(entorno):
    assert entorno.esperar(lambda: len(entorno.aperturas) == 1)
    entorno.puerto.fallar = True
    assert entorno.esperar(lambda: entorno.puerto.cerrado)
    entorno.puerto.fallar = False
    assert entorno.esperar(lambda: len(entorno.aperturas) >= 2)


def test_al_destruir_el_nodo_frena_y_desarma(ros):
    puerto = PuertoFalso()
    nodo = HardwareBridgeNode(port_factory=lambda ruta, baud: puerto)
    executor = SingleThreadedExecutor()
    executor.add_node(nodo)
    limite = time.monotonic() + TIMEOUT_S
    while not puerto.escritos and time.monotonic() < limite:
        executor.spin_once(timeout_sec=0.05)
    executor.shutdown()
    nodo.destroy_node()
    assert puerto.tramas()[-2:] == ['C:SET_SPEED:0', 'M:SYS_DISARM:0']
    assert puerto.cerrado
