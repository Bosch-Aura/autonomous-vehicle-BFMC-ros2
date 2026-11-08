"""Tests del nodo de detección de carril: verifican la conexión entre imagen y LaneState.

Usan los parámetros reales de config/params.yaml. Esperan mensajes reales, por eso
tienen un tope de tiempo; la visión se prueba aparte, sin ROS, en
test_lane_detection_logic.py y test_lane_detection_yaml.py.
"""
import gc
import math
from pathlib import Path
import threading
import time

from bfmc_interfaces.msg import HardwareStatus, LaneState
from bfmc_lane_detection.ground_calibration import lane_config_desde_parametros
from bfmc_lane_detection.lane_detection_logic import LaneDetector
from bfmc_lane_detection.lane_detection_node import LaneDetectionNode
from bfmc_lane_detection.param_store import leer_parametros
import pytest
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from synthetic_depth import (
    a_16uc1, ALTO, ANCHO, CX, CY, FX, FY, pared, profundidad_del_piso,
)
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
    detector = LaneDetector(lane_config_desde_parametros(parametros))
    ancho, alto, carril = (
        parametros['warp_width'], parametros['warp_height'], detector.lane_width_px)
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
    # Sin ninguna medición válida antes, no hay nada que predecir: carril perdido.
    assert estado.mode == LaneState.MODE_LOST
    assert math.isnan(estado.lateral_error_m)


def test_un_carril_bueno_se_publica_en_modo_dos_lineas(entorno, parametros):
    estado = publicar_y_esperar(
        entorno, mensaje_de(carril_visto_por_la_camara(parametros, corrimiento_px=0)))
    assert estado.mode == LaneState.MODE_TWO_LINES
    assert (estado.left_valid, estado.right_valid) == (True, True)


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


def test_un_cambio_del_rectangulo_rehace_el_trapecio_y_la_escala(entorno_completo):
    nodo, *_ = entorno_completo
    anterior = nodo._detector
    resultado = nodo.set_parameters_atomically([
        Parameter('calib_forward_near_m', Parameter.Type.DOUBLE, 0.50),
        Parameter('calib_half_width_m', Parameter.Type.DOUBLE, 0.22)])
    assert resultado.successful, resultado.reason
    assert nodo._detector is not anterior
    assert nodo._detector.config.warp_src != anterior.config.warp_src
    assert nodo._detector.config.view_width_m == pytest.approx(0.44)
    assert nodo._detector.config.view_length_m == pytest.approx(0.45)


def test_el_ancho_del_carril_no_cambia_el_trapecio_ni_la_escala(entorno_completo):
    nodo, *_ = entorno_completo
    anterior = nodo._detector.config
    assert nodo.set_parameters(
        [Parameter('lane_width_m', Parameter.Type.DOUBLE, 0.40)])[0].successful
    nuevo = nodo._detector.config
    assert nuevo.lane_width_m == pytest.approx(0.40)
    assert (nuevo.warp_src, nuevo.view_width_m, nuevo.view_length_m) == (
        anterior.warp_src, anterior.view_width_m, anterior.view_length_m)


def test_warp_src_ya_no_es_un_parametro(entorno_completo):
    nodo, *_ = entorno_completo
    assert not nodo.has_parameter('warp_src')
    assert not nodo.has_parameter('lane_width_px')
    assert not nodo.has_parameter('forward_range_m')


def test_un_cambio_de_umbral_de_color_llega_al_detector(entorno_completo):
    nodo, *_ = entorno_completo
    resultado = nodo.set_parameters(
        [Parameter('white_hsv_min', Parameter.Type.INTEGER_ARRAY, [0, 0, 240])])
    assert resultado[0].successful
    assert tuple(nodo._detector.config.white_hsv_min) == (0, 0, 240)


@pytest.mark.parametrize('nombre, valor', [
    ('calib_half_width_m', 0.6),       # el borde lejano del rectángulo no entra en la imagen
    ('calib_forward_near_m', 2.0),     # el borde cercano más lejos que el lejano
    ('calib_pitch_deg', -10.0),        # mirando para arriba: el piso no se ve
    ('lane_width_m', 0.6),             # el carril no entra en la vista
])
def test_una_geometria_invalida_se_rechaza_y_el_detector_sigue_igual(
        entorno_completo, nombre, valor):
    nodo, *_ = entorno_completo
    anterior = nodo._detector
    valor_anterior = nodo.get_parameter(nombre).value
    resultado = nodo.set_parameters([Parameter(nombre, Parameter.Type.DOUBLE, valor)])
    assert resultado[0].successful is False
    assert resultado[0].reason
    assert nodo._detector is anterior
    assert nodo.get_parameter(nombre).value == valor_anterior


# --- con el auto andando --------------------------------------------------------------

def avisar_estado(entorno, estado):
    """Publica el estado del ESP32 hasta que el nodo lo recibe."""
    nodo, executor, *_ = entorno
    auxiliar = rclpy.create_node('auxiliar_de_estado')
    executor.add_node(auxiliar)
    publicador = auxiliar.create_publisher(
        HardwareStatus, nodo.get_parameter('hardware_status_topic').value,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
    try:
        limite = time.monotonic() + TIMEOUT_S
        while nodo._estado_esp32 != estado and time.monotonic() < limite:
            publicador.publish(HardwareStatus(esp32_state=estado))
            executor.spin_once(timeout_sec=0.1)
        assert nodo._estado_esp32 == estado
    finally:
        executor.remove_node(auxiliar)
        auxiliar.destroy_node()


def test_con_el_auto_andando_no_se_puede_cambiar_la_geometria(entorno_completo):
    nodo, *_ = entorno_completo
    avisar_estado(entorno_completo, 'RUNNING')
    resultado = nodo.set_parameters(
        [Parameter('calib_forward_near_m', Parameter.Type.DOUBLE, 0.50)])
    assert resultado[0].successful is False
    assert 'andando' in resultado[0].reason


def test_con_el_auto_andando_el_ancho_del_carril_si_se_puede_cambiar(entorno_completo):
    nodo, *_ = entorno_completo
    avisar_estado(entorno_completo, 'RUNNING')
    assert nodo.set_parameters(
        [Parameter('lane_width_m', Parameter.Type.DOUBLE, 0.40)])[0].successful


def test_con_el_auto_andando_no_se_puede_calibrar(entorno_completo):
    nodo, *_ = entorno_completo
    avisar_estado(entorno_completo, 'RUNNING')
    captura_antes = nodo._captura     # la del chequeo al arrancar, si sigue en curso
    respuesta = nodo._al_calibrar(Trigger.Request(), Trigger.Response())
    assert respuesta.success is False
    assert 'andando' in respuesta.message
    assert nodo._captura is captura_antes    # ni siquiera empezó a juntar cuadros


def test_con_el_auto_detenido_se_puede_cambiar_la_geometria(entorno_completo):
    nodo, *_ = entorno_completo
    avisar_estado(entorno_completo, 'STOPPED')
    assert nodo.set_parameters(
        [Parameter('calib_forward_near_m', Parameter.Type.DOUBLE, 0.50)])[0].successful


# --- guardar ----------------------------------------------------------------------------

def test_guardar_escribe_las_medidas_y_la_geometria_fuera_del_repo(ros, tmp_path):
    archivo = tmp_path / 'config' / 'lane_detection.yaml'
    nodo = LaneDetectionNode(parameter_overrides=[
        Parameter('persist_file', Parameter.Type.STRING, str(archivo))])
    try:
        nodo.set_parameters([Parameter('lane_width_m', Parameter.Type.DOUBLE, 0.385)])
        respuesta = nodo._al_guardar(Trigger.Request(), Trigger.Response())
        assert respuesta.success, respuesta.message
        guardado = leer_parametros(str(archivo), 'lane_detection')
        assert guardado['lane_width_m'] == pytest.approx(0.385)
        assert guardado['calib_height_m'] == nodo.get_parameter('calib_height_m').value
        assert guardado['camera_intrinsics'] == list(
            nodo.get_parameter('camera_intrinsics').value)
        # Solo la lista fija: ni topics ni parámetros de la calibración.
        assert 'lane_topic' not in guardado and 'calib_frames' not in guardado
    finally:
        nodo.destroy_node()


def test_lo_guardado_arma_un_detector_valido(ros, tmp_path, parametros):
    archivo = tmp_path / 'lane_detection.yaml'
    nodo = LaneDetectionNode(parameter_overrides=[
        Parameter('persist_file', Parameter.Type.STRING, str(archivo))])
    try:
        nodo.set_parameters([Parameter('calib_half_width_m', Parameter.Type.DOUBLE, 0.22)])
        assert nodo._al_guardar(Trigger.Request(), Trigger.Response()).success
        config = lane_config_desde_parametros(
            {**parametros, **leer_parametros(str(archivo), 'lane_detection')})
        assert config.view_width_m == pytest.approx(0.44)
    finally:
        nodo.destroy_node()


def test_si_no_se_puede_escribir_guardar_responde_el_motivo(ros, tmp_path):
    bloqueo = tmp_path / 'no-es-carpeta'
    bloqueo.write_text('')
    nodo = LaneDetectionNode(parameter_overrides=[
        Parameter('persist_file', Parameter.Type.STRING, str(bloqueo / 'lane.yaml'))])
    try:
        respuesta = nodo._al_guardar(Trigger.Request(), Trigger.Response())
        assert respuesta.success is False
        assert 'no se pudo guardar' in respuesta.message
    finally:
        nodo.destroy_node()


def test_los_topics_no_se_pueden_cambiar_con_el_nodo_andando(entorno_completo):
    nodo, *_ = entorno_completo
    resultado = nodo.set_parameters([Parameter('lane_topic', Parameter.Type.STRING, '/otro')])
    assert resultado[0].successful is False
    assert 'arrancar' in resultado[0].reason


def test_la_frecuencia_de_depuracion_debe_ser_positiva(entorno_completo):
    nodo, *_ = entorno_completo
    resultado = nodo.set_parameters([Parameter('debug_max_fps', Parameter.Type.DOUBLE, 0.0)])
    assert resultado[0].successful is False


# --- servicio de calibración ----------------------------------------------------------

@pytest.fixture
def con_profundidad(ros):
    """El nodo corriendo en varios hilos (como en main) y un auxiliar que publica profundidad.

    `profundidad[0]` es el mensaje de profundidad que se publica: el test lo cambia con
    `mensaje_de_profundidad`. Los mensajes se arman una sola vez: asignar 600 KB a
    `Image.data` en Python es lento, y armarlos en cada vuelta del timer lo satura.
    """
    nodo = LaneDetectionNode()
    auxiliar = rclpy.create_node('auxiliar_de_profundidad')
    calidad = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    publicador = auxiliar.create_publisher(Image, nodo.get_parameter('depth_topic').value, calidad)
    publicador_info = auxiliar.create_publisher(
        CameraInfo, nodo.get_parameter('depth_info_topic').value, calidad)
    profundidad = [mensaje_de_profundidad(profundidad_del_piso(0.20, 20.0))]
    info = CameraInfo()
    info.height, info.width = ALTO, ANCHO
    info.k = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]

    def publicar():
        publicador.publish(profundidad[0])
        publicador_info.publish(info)

    auxiliar.create_timer(1.0 / 15.0, publicar)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(nodo)
    executor.add_node(auxiliar)
    hilo = threading.Thread(target=executor.spin, daemon=True)
    hilo.start()
    yield nodo, auxiliar, profundidad
    executor.shutdown()
    hilo.join(timeout=TIMEOUT_S)
    nodo.destroy_node()
    auxiliar.destroy_node()


def mensaje_de_profundidad(metros):
    mensaje = Image()
    mensaje.height, mensaje.width = ALTO, ANCHO
    mensaje.encoding = '16UC1'
    mensaje.step = ANCHO * 2
    mensaje.data = a_16uc1(metros).tobytes()
    return mensaje


def calibrar(nodo, auxiliar):
    # El cliente en su propio grupo: así su respuesta no espera detrás del timer que publica.
    cliente = auxiliar.create_client(Trigger, '/lane_detection/calibrate_ground',
                                     callback_group=MutuallyExclusiveCallbackGroup())
    try:
        assert cliente.wait_for_service(timeout_sec=TIMEOUT_S)
        futuro = cliente.call_async(Trigger.Request())
        limite = time.monotonic() + 3 * TIMEOUT_S
        while not futuro.done() and time.monotonic() < limite:
            time.sleep(0.05)
        assert futuro.done(), 'el servicio no respondió'
        return futuro.result()
    finally:
        auxiliar.destroy_client(cliente)


GEOMETRIA = ('calib_height_m', 'calib_pitch_deg', 'calib_roll_deg', 'camera_intrinsics')


def geometria(nodo):
    return {nombre: nodo.get_parameter(nombre).value for nombre in GEOMETRIA}


def test_calibrar_con_el_piso_a_la_vista_aplica_la_pose_medida(con_profundidad):
    nodo, auxiliar, _ = con_profundidad
    # Una pose vieja distinta de la real (0,20 m y 20°), para ver que la calibración la cambia.
    assert nodo.set_parameters_atomically([
        Parameter('calib_height_m', Parameter.Type.DOUBLE, 0.25),
        Parameter('calib_pitch_deg', Parameter.Type.DOUBLE, 25.0)]).successful
    respuesta = calibrar(nodo, auxiliar)
    assert respuesta.success, respuesta.message
    assert 'altura 0.20' in respuesta.message and 'pitch 20.' in respuesta.message
    assert '1 px = 1.25 mm' in respuesta.message
    actual = geometria(nodo)
    assert actual['calib_height_m'] == pytest.approx(0.20, abs=0.005)
    assert actual['calib_pitch_deg'] == pytest.approx(20.0, abs=0.5)
    assert list(actual['camera_intrinsics']) == [FX, FY, CX, CY, float(ANCHO), float(ALTO)]
    assert nodo._detector.config.view_width_m == pytest.approx(0.5)


def test_con_el_piso_tapado_responde_el_motivo_y_no_cambia_nada(con_profundidad):
    nodo, auxiliar, profundidad = con_profundidad
    profundidad[0] = mensaje_de_profundidad(pared(0.4))
    antes = geometria(nodo)
    detector = nodo._detector
    respuesta = calibrar(nodo, auxiliar)
    assert respuesta.success is False
    assert 'despejá la zona' in respuesta.message
    assert {k: list(v) if k == 'camera_intrinsics' else v for k, v in geometria(nodo).items()} \
        == {k: list(v) if k == 'camera_intrinsics' else v for k, v in antes.items()}
    assert nodo._detector is detector


def test_sin_profundidad_responde_que_revise_la_camara(ros):
    nodo = LaneDetectionNode()
    nodo.set_parameters([Parameter('calib_timeout_s', Parameter.Type.DOUBLE, 0.5)])
    auxiliar = rclpy.create_node('auxiliar_sin_profundidad')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(nodo)
    executor.add_node(auxiliar)
    hilo = threading.Thread(target=executor.spin, daemon=True)
    hilo.start()
    try:
        respuesta = calibrar(nodo, auxiliar)
        assert respuesta.success is False
        assert 'align_depth' in respuesta.message
        assert nodo._captura is None   # no queda una captura en curso (lo que llegue se ignora)
    finally:
        executor.shutdown()
        hilo.join(timeout=TIMEOUT_S)
        nodo.destroy_node()
        auxiliar.destroy_node()


def test_despues_de_calibrar_no_queda_una_captura_en_curso(con_profundidad):
    # Las suscripciones a la profundidad quedan (destruirlas con el executor andando tira
    # el nodo abajo), pero sin captura en curso lo que llega se ignora.
    nodo, auxiliar, _ = con_profundidad
    calibrar(nodo, auxiliar)
    assert nodo._captura is None


@pytest.mark.parametrize('nombre, valor', [
    ('calib_inlier_m', 0.0), ('calib_frames', 0), ('calib_forward_near_m', 5.0)])
def test_un_parametro_de_calibracion_invalido_se_rechaza(entorno_completo, nombre, valor):
    nodo, *_ = entorno_completo
    tipo = Parameter.Type.INTEGER if isinstance(valor, int) else Parameter.Type.DOUBLE
    assert nodo.set_parameters([Parameter(nombre, tipo, valor)])[0].successful is False


def test_el_servicio_de_calibracion_no_se_puede_cambiar_con_el_nodo_andando(entorno_completo):
    nodo, *_ = entorno_completo
    resultado = nodo.set_parameters(
        [Parameter('calibrate_service', Parameter.Type.STRING, '/otro')])
    assert resultado[0].successful is False
