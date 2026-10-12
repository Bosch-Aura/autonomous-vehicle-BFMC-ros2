"""Tests del nodo de detección de carril: verifican la conexión entre imagen y LaneState.

Usan los parámetros reales de config/params.yaml. Esperan mensajes reales, por eso
tienen un tope de tiempo; la visión se prueba aparte, sin ROS, en
test_lane_detection_logic.py y test_lane_detection_yaml.py.
"""
import gc
from pathlib import Path
import time

from bfmc_interfaces.msg import LaneState
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from synthetic_lanes import a_vista_de_camara, carril_cenital, lienzo
import yaml

from bfmc_lane_detection.lane_detection_logic import LaneConfig, LaneDetector
from bfmc_lane_detection.lane_detection_node import LaneDetectionNode

PARAMS = Path(__file__).resolve().parents[1] / 'config' / 'params.yaml'
TIMEOUT_S = 5.0
CAMARA_ANCHO = 640
CAMARA_ALTO = 480


@pytest.fixture(scope='module')
def parametros():
    with open(PARAMS, encoding='utf-8') as archivo:
        return yaml.safe_load(archivo)['lane_detection']['ros__parameters']


@pytest.fixture(scope='module')
def ros():
    rclpy.init(args=['--ros-args', '--params-file', str(PARAMS)])
    yield
    # Se liberan los objetos de rclpy antes de cerrarlo, para evitar un fallo de segmentación.
    gc.collect()
    rclpy.shutdown()


@pytest.fixture
def entorno(ros):
    nodo = LaneDetectionNode()
    auxiliar = rclpy.create_node('auxiliar_de_test_lane')
    executor = SingleThreadedExecutor()
    executor.add_node(nodo)
    executor.add_node(auxiliar)
    estados = []
    auxiliar.create_subscription(
        LaneState, nodo.get_parameter('lane_topic').value, estados.append,
        QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
    publicador = auxiliar.create_publisher(
        Image, nodo.get_parameter('image_topic').value,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    yield estados, publicador, executor
    executor.shutdown()
    nodo.destroy_node()
    auxiliar.destroy_node()


def mensaje_de(imagen, codificacion='bgr8'):
    mensaje = Image()
    mensaje.header.frame_id = 'camara_de_prueba'
    mensaje.height, mensaje.width = imagen.shape[:2]
    mensaje.encoding = codificacion
    mensaje.step = mensaje.width * 3
    mensaje.data = imagen.tobytes()
    return mensaje


def carril_visto_por_la_camara(parametros, corrimiento_px):
    detector = LaneDetector(LaneConfig(
        **{k: parametros[k] for k in LaneConfig.__dataclass_fields__}))
    ancho, alto, carril = (
        parametros['warp_width'], parametros['warp_height'], parametros['lane_width_px'])
    centro = ancho / 2 + corrimiento_px
    cenital = carril_cenital(
        ancho, alto, int(centro - carril / 2), int(centro + carril / 2), grosor=10)
    return a_vista_de_camara(
        cenital, detector.warp_matrix(CAMARA_ANCHO, CAMARA_ALTO), CAMARA_ANCHO, CAMARA_ALTO)


def publicar_y_esperar(entorno, mensaje):
    estados, publicador, executor = entorno
    limite = time.monotonic() + TIMEOUT_S
    while not estados and time.monotonic() < limite:
        publicador.publish(mensaje)
        executor.spin_once(timeout_sec=0.1)
    return estados[0] if estados else None


def test_una_imagen_con_el_carril_a_la_derecha_publica_desvio_negativo(entorno, parametros):
    estado = publicar_y_esperar(
        entorno, mensaje_de(carril_visto_por_la_camara(parametros, corrimiento_px=40)))
    assert estado is not None
    assert estado.detected is True
    assert estado.lateral_error_m < -0.02
    assert estado.confidence > 0.0


def test_una_imagen_con_el_carril_a_la_izquierda_publica_desvio_positivo(entorno, parametros):
    estado = publicar_y_esperar(
        entorno, mensaje_de(carril_visto_por_la_camara(parametros, corrimiento_px=-40)))
    assert estado.detected is True
    assert estado.lateral_error_m > 0.02


def test_una_imagen_sin_carril_publica_no_detectado(entorno):
    estado = publicar_y_esperar(entorno, mensaje_de(lienzo(CAMARA_ANCHO, CAMARA_ALTO)))
    assert estado.detected is False
    assert estado.confidence == 0.0


def test_el_encabezado_del_resultado_es_el_de_la_imagen(entorno):
    estado = publicar_y_esperar(entorno, mensaje_de(lienzo(CAMARA_ANCHO, CAMARA_ALTO)))
    assert estado.header.frame_id == 'camara_de_prueba'


def test_una_imagen_rgb_se_interpreta_igual_que_una_bgr(entorno, parametros):
    bgr = carril_visto_por_la_camara(parametros, corrimiento_px=40)
    estado = publicar_y_esperar(entorno, mensaje_de(bgr[:, :, ::-1].copy(), 'rgb8'))
    assert estado.detected is True
    assert estado.lateral_error_m < -0.02


def test_una_imagen_con_codificacion_no_soportada_se_descarta_sin_publicar(entorno):
    estados, publicador, executor = entorno
    publicador.publish(mensaje_de(lienzo(CAMARA_ANCHO, CAMARA_ALTO), 'mono8'))
    limite = time.monotonic() + 1.0
    while time.monotonic() < limite:
        executor.spin_once(timeout_sec=0.1)
    assert estados == []
