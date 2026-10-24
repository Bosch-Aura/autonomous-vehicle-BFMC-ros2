"""Tests del nodo de detección de carril: verifican la conexión entre imagen y LaneState.

Usan los parámetros reales de config/params.yaml. Esperan mensajes reales, por eso
tienen un tope de tiempo; la visión se prueba aparte, sin ROS, en
test_lane_detection_logic.py y test_lane_detection_yaml.py.
"""
import gc
from pathlib import Path
import time

from bfmc_interfaces.msg import LaneState
from bfmc_lane_detection.lane_detection_logic import LaneConfig, LaneDetector
from bfmc_lane_detection.lane_detection_node import LaneDetectionNode
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from synthetic_lanes import a_vista_de_camara, carril_cenital, lienzo
import yaml

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
    # Cada paquete usa su propio dominio de ROS: colcon corre los tests de varios paquetes a la
    # vez y, en el mismo dominio, se cruzarían los mensajes de topics con el mismo nombre.
    rclpy.init(args=['--ros-args', '--params-file', str(PARAMS)], domain_id=16)
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


# --- imágenes de depuración y cambios en vivo --------------------------------------

@pytest.fixture
def entorno_completo(ros):
    nodo = LaneDetectionNode()
    auxiliar = rclpy.create_node('auxiliar_de_test_lane_completo')
    executor = SingleThreadedExecutor()
    executor.add_node(nodo)
    executor.add_node(auxiliar)
    estados, camara, cenital = [], [], []
    calidad = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    auxiliar.create_subscription(
        LaneState, nodo.get_parameter('lane_topic').value, estados.append, calidad)
    auxiliar.create_subscription(
        Image, nodo.get_parameter('debug_overlay_topic').value, camara.append, calidad)
    auxiliar.create_subscription(
        Image, nodo.get_parameter('debug_birdview_topic').value, cenital.append, calidad)
    publicador = auxiliar.create_publisher(
        Image, nodo.get_parameter('image_topic').value,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    yield nodo, executor, publicador, estados, camara, cenital
    executor.shutdown()
    nodo.destroy_node()
    auxiliar.destroy_node()


def publicar_hasta(entorno, mensaje, condicion, segundos=TIMEOUT_S):
    _, executor, publicador, *_ = entorno
    limite = time.monotonic() + segundos
    while not condicion() and time.monotonic() < limite:
        publicador.publish(mensaje)
        executor.spin_once(timeout_sec=0.1)
    return condicion()


def test_con_suscriptores_se_publican_las_dos_imagenes_de_depuracion(entorno_completo, parametros):
    _, _, _, estados, camara, cenital = entorno_completo
    mensaje = mensaje_de(carril_visto_por_la_camara(parametros, 0))
    assert publicar_hasta(entorno_completo, mensaje, lambda: camara and cenital and estados)
    assert (camara[0].height, camara[0].width) == (CAMARA_ALTO, CAMARA_ANCHO)
    assert (cenital[0].height, cenital[0].width) == (
        parametros['warp_height'], parametros['warp_width'])
    assert camara[0].encoding == cenital[0].encoding == 'bgr8'
    assert camara[0].header.frame_id == 'camara_de_prueba'
    assert len(camara[0].data) == camara[0].step * camara[0].height


def test_con_publish_debug_apagado_no_se_publican_imagenes_de_depuracion(
        entorno_completo, parametros):
    nodo, _, _, estados, camara, cenital = entorno_completo
    resultado = nodo.set_parameters([Parameter('publish_debug', Parameter.Type.BOOL, False)])
    assert resultado[0].successful
    mensaje = mensaje_de(carril_visto_por_la_camara(parametros, 0))
    assert publicar_hasta(entorno_completo, mensaje, lambda: estados)
    publicar_hasta(entorno_completo, mensaje, lambda: False, segundos=1.0)
    assert camara == [] and cenital == []


def test_las_imagenes_de_depuracion_no_cambian_el_resultado(entorno_completo, parametros):
    nodo, _, _, estados, camara, _ = entorno_completo
    mensaje = mensaje_de(carril_visto_por_la_camara(parametros, 30))
    assert publicar_hasta(entorno_completo, mensaje, lambda: camara and estados)
    con_depuracion = estados[-1].lateral_error_m
    nodo.set_parameters([Parameter('publish_debug', Parameter.Type.BOOL, False)])
    estados.clear()
    assert publicar_hasta(entorno_completo, mensaje, lambda: estados)
    assert estados[-1].lateral_error_m == pytest.approx(con_depuracion)


def test_un_cambio_valido_de_warp_src_rehace_el_detector(entorno_completo):
    nodo, *_ = entorno_completo
    nuevo = [0.2, 0.5, 0.8, 0.5, 1.0, 1.0, 0.0, 1.0]
    anterior = nodo._detector
    resultado = nodo.set_parameters([Parameter('warp_src', Parameter.Type.DOUBLE_ARRAY, nuevo)])
    assert resultado[0].successful
    assert nodo._detector is not anterior
    assert list(nodo._detector.config.warp_src) == nuevo
    assert list(nodo.get_parameter('warp_src').value) == nuevo


def test_un_cambio_de_umbral_de_color_llega_al_detector(entorno_completo):
    nodo, *_ = entorno_completo
    resultado = nodo.set_parameters(
        [Parameter('white_hsv_min', Parameter.Type.INTEGER_ARRAY, [0, 0, 240])])
    assert resultado[0].successful
    assert tuple(nodo._detector.config.white_hsv_min) == (0, 0, 240)


@pytest.mark.parametrize('invalido', [
    [0.0, 0.5, 1.0, 0.5, 1.0, 0.5, 0.0, 0.5],   # puntos alineados
    [0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 1.0],   # lados cruzados
    [0.0, 0.0, 1.0, 0.0, 1.0, 1.0],             # faltan valores
    [0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 2.0],   # fuera de la imagen
])
def test_un_warp_src_invalido_se_rechaza_y_el_detector_sigue_igual(entorno_completo, invalido):
    nodo, *_ = entorno_completo
    anterior = nodo._detector
    valor_anterior = list(nodo.get_parameter('warp_src').value)
    resultado = nodo.set_parameters([Parameter('warp_src', Parameter.Type.DOUBLE_ARRAY, invalido)])
    assert resultado[0].successful is False
    assert resultado[0].reason
    assert nodo._detector is anterior
    assert list(nodo.get_parameter('warp_src').value) == valor_anterior


def test_los_topics_no_se_pueden_cambiar_con_el_nodo_andando(entorno_completo):
    nodo, *_ = entorno_completo
    resultado = nodo.set_parameters([Parameter('lane_topic', Parameter.Type.STRING, '/otro')])
    assert resultado[0].successful is False
    assert 'arrancar' in resultado[0].reason


def test_la_frecuencia_de_depuracion_debe_ser_positiva(entorno_completo):
    nodo, *_ = entorno_completo
    resultado = nodo.set_parameters([Parameter('debug_max_fps', Parameter.Type.DOUBLE, 0.0)])
    assert resultado[0].successful is False
