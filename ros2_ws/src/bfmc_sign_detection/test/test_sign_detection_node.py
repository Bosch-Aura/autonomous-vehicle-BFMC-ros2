"""Tests del nodo de señales con un detector falso: no necesitan el modelo ni YOLO.

Usan los parámetros reales de config/params.yaml. Esperan mensajes reales, por eso
tienen un tope de tiempo; las reglas se prueban aparte, sin ROS, en
test_sign_detection_logic.py.
"""
import gc
import math
from pathlib import Path
import time

from bfmc_interfaces.msg import SignDetectionArray
import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from bfmc_sign_detection.sign_detection_logic import RawDetection
from bfmc_sign_detection.sign_detection_node import SignDetectionNode

PARAMS = Path(__file__).resolve().parents[1] / 'config' / 'params.yaml'
TIMEOUT_S = 5.0
ANCHO = 200
ALTO = 100
BEST_EFFORT = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
RELIABLE = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)


class DetectorFalso:
    def __init__(self, detecciones):
        self.detecciones = detecciones

    def detect(self, bgr):
        return self.detecciones


@pytest.fixture(scope='module')
def ros():
    # Cada paquete usa su propio dominio de ROS: colcon corre los tests de varios paquetes a la
    # vez y, en el mismo dominio, se cruzarían los mensajes de topics con el mismo nombre.
    rclpy.init(args=['--ros-args', '--params-file', str(PARAMS)], domain_id=17)
    yield
    # Se liberan los objetos de rclpy antes de cerrarlo, para evitar un fallo de segmentación.
    gc.collect()
    rclpy.shutdown()


class Entorno:
    def __init__(self, detecciones):
        self.fabrica_llamadas = []

        def fabrica(ruta, dispositivo, confianza):
            self.fabrica_llamadas.append((ruta, dispositivo, confianza))
            return DetectorFalso(detecciones)

        self.nodo = SignDetectionNode(detector_factory=fabrica)
        self.auxiliar = rclpy.create_node('auxiliar_de_test_signs')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.nodo)
        self.executor.add_node(self.auxiliar)
        self.salidas = []
        param = self.nodo.get_parameter
        self.pub_color = self.auxiliar.create_publisher(
            Image, param('color_topic').value, BEST_EFFORT)
        self.pub_profundidad = self.auxiliar.create_publisher(
            Image, param('depth_topic').value, BEST_EFFORT)
        self.auxiliar.create_subscription(
            SignDetectionArray, param('signs_topic').value, self.salidas.append, RELIABLE)

    def cerrar(self):
        self.executor.shutdown()
        self.nodo.destroy_node()
        self.auxiliar.destroy_node()

    def publicar_y_esperar(self, con_profundidad, milimetros=2000):
        color = Image()
        color.header.frame_id = 'camara'
        color.height, color.width, color.encoding = ALTO, ANCHO, 'bgr8'
        color.step = ANCHO * 3
        color.data = bytes(ALTO * ANCHO * 3)

        profundidad = Image()
        profundidad.height, profundidad.width, profundidad.encoding = ALTO, ANCHO, '16UC1'
        profundidad.step = ANCHO * 2
        profundidad.data = np.full((ALTO, ANCHO), milimetros, dtype='<u2').tobytes()

        if con_profundidad:
            # La profundidad tiene que llegar antes que el color: si no, la primera
            # imagen se procesa sin ella y su distancia sale desconocida, como debe ser.
            self._esperar_profundidad(profundidad)

        limite = time.monotonic() + TIMEOUT_S
        while not self.salidas and time.monotonic() < limite:
            if con_profundidad:
                self.pub_profundidad.publish(profundidad)
            self.pub_color.publish(color)
            self._girar()
        return self.salidas[0] if self.salidas else None

    def _esperar_profundidad(self, profundidad):
        limite = time.monotonic() + TIMEOUT_S
        while self.nodo._profundidad is None and time.monotonic() < limite:
            self.pub_profundidad.publish(profundidad)
            for _ in range(10):
                self.executor.spin_once(timeout_sec=0.02)

    def _girar(self, vueltas=10):
        """Atiende varios callbacks: cada spin_once atiende uno solo, y con dos mensajes de
        entrada pendientes el del suscriptor de resultados quedaría sin turno."""
        for _ in range(vueltas):
            self.executor.spin_once(timeout_sec=0.02)
            if self.salidas:
                return


@pytest.fixture
def stop_cercano(ros):
    entorno = Entorno([RawDetection('stop', 0.9, 80, 30, 120, 70)])
    yield entorno
    entorno.cerrar()


@pytest.fixture
def sin_detecciones(ros):
    entorno = Entorno([])
    yield entorno
    entorno.cerrar()


@pytest.fixture
def clase_desconocida(ros):
    entorno = Entorno([RawDetection('semaforo_raro', 0.99, 80, 30, 120, 70)])
    yield entorno
    entorno.cerrar()


def test_el_nodo_crea_el_detector_con_el_modelo_y_el_umbral_del_yaml(stop_cercano):
    ruta, dispositivo, confianza = stop_cercano.fabrica_llamadas[0]
    assert ruta == stop_cercano.nodo.get_parameter('model_path').value
    assert confianza == stop_cercano.nodo.get_parameter('yolo_confidence').value


def test_una_senal_con_profundidad_se_publica_con_su_distancia(stop_cercano):
    salida = stop_cercano.publicar_y_esperar(con_profundidad=True, milimetros=2000)
    assert len(salida.detections) == 1
    senal = salida.detections[0]
    assert senal.label == 'stop'
    assert senal.distance_m == pytest.approx(2.0, abs=1e-3)
    assert senal.confidence == pytest.approx(0.9, abs=1e-5)
    assert (senal.x_min, senal.y_min, senal.x_max, senal.y_max) == (80, 30, 120, 70)


def test_sin_profundidad_la_senal_se_publica_con_distancia_desconocida(stop_cercano):
    salida = stop_cercano.publicar_y_esperar(con_profundidad=False)
    assert salida.detections[0].label == 'stop'
    assert math.isnan(salida.detections[0].distance_m)


def test_sin_detecciones_se_publica_un_mensaje_vacio_para_indicar_que_sigue_vivo(
        sin_detecciones):
    salida = sin_detecciones.publicar_y_esperar(con_profundidad=True)
    assert list(salida.detections) == []


def test_una_clase_que_no_esta_en_el_mapa_no_se_publica(clase_desconocida):
    salida = clase_desconocida.publicar_y_esperar(con_profundidad=True)
    assert list(salida.detections) == []


def test_el_encabezado_del_resultado_es_el_de_la_imagen_de_color(stop_cercano):
    assert stop_cercano.publicar_y_esperar(con_profundidad=True).header.frame_id == 'camara'
