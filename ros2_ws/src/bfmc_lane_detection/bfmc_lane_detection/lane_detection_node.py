"""Nodo ROS2 de bfmc_lane_detection: convierte las imágenes de la cámara en LaneState."""
import dataclasses

from bfmc_interfaces.msg import LaneState
from bfmc_lane_detection.image_utils import imagen_a_bgr
from bfmc_lane_detection.lane_debug import dibujar_camara, dibujar_cenital
from bfmc_lane_detection.lane_detection_logic import LaneConfig, LaneDetector
import cv2
import numpy as np
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

PARAMETROS_TEXTO = ('image_topic', 'lane_topic', 'debug_overlay_topic', 'debug_birdview_topic')
PARAMETROS_ENTEROS = (
    'morph_kernel_px', 'warp_width', 'warp_height', 'n_windows', 'window_margin_px',
    'min_pixels', 'full_confidence_pixels', 'intersection_min_rows',
)
PARAMETROS_DECIMALES = (
    'roi_top_fraction', 'lane_width_m', 'lane_width_px', 'forward_range_m',
    'reference_row_fraction', 'intersection_band_top_fraction', 'intersection_row_fraction',
    'debug_max_fps',
)
PARAMETROS_ARREGLOS = ('white_hsv_min', 'white_hsv_max', 'warp_src')

# Los topics solo se eligen al arrancar: cambiarlos en vivo obligaría a rehacer las
# suscripciones y publicaciones. Todo lo demás se puede cambiar con el nodo andando.
PARAMETROS_SOLO_AL_ARRANCAR = (
    'image_topic', 'lane_topic', 'debug_overlay_topic', 'debug_birdview_topic')
CAMPOS_DE_CONFIG = tuple(campo.name for campo in dataclasses.fields(LaneConfig))


class LaneDetectionNode(Node):
    """Lee las imágenes de color y publica el estado del carril en /perception/lane.

    Cada imagen se procesa sola. El encabezado del resultado es el de la imagen, así
    quien lo consume sabe de qué instante es la medición.

    Todos los parámetros se declaran sin valor por defecto: los valores viven en
    config/params.yaml y, si falta alguno, el nodo falla al arrancar.

    Los parámetros de la visión se pueden cambiar con el nodo andando (por ejemplo con
    `ros2 param set` o desde el visor de calibración). El cambio se valida antes de
    aplicarse: un valor inválido se rechaza y el detector sigue como estaba. No se guarda
    en el YAML. El nodo corre en un solo hilo, así que el cambio nunca se cruza con una
    imagen a medio procesar.

    Si hay alguien suscripto a las imágenes de depuración, además publica lo que ve el
    detector (ver lane_debug.py), a lo sumo `debug_max_fps` veces por segundo.
    """

    def __init__(self):
        super().__init__('lane_detection')

        for nombre in PARAMETROS_TEXTO:
            self.declare_parameter(nombre, Parameter.Type.STRING)
        for nombre in PARAMETROS_ENTEROS:
            self.declare_parameter(nombre, Parameter.Type.INTEGER)
        for nombre in PARAMETROS_DECIMALES:
            self.declare_parameter(nombre, Parameter.Type.DOUBLE)
        self.declare_parameter('publish_debug', Parameter.Type.BOOL)
        self.declare_parameter('white_hsv_min', Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter('white_hsv_max', Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter('warp_src', Parameter.Type.DOUBLE_ARRAY)

        self._detector = LaneDetector(self._configuracion())

        # Las imágenes se leen con calidad "best effort" y cola de 1: si el nodo se
        # atrasa, conviene procesar la imagen más nueva y no acumular retraso.
        imagenes = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        resultados = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        # Las de depuración son "reliable": las acepta tanto un suscriptor "reliable" como
        # uno "best effort".
        depuracion = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self._publisher = self.create_publisher(
            LaneState, self._valor('lane_topic'), resultados)
        self._publicador_camara = self.create_publisher(
            Image, self._valor('debug_overlay_topic'), depuracion)
        self._publicador_cenital = self.create_publisher(
            Image, self._valor('debug_birdview_topic'), depuracion)
        self._subscription = self.create_subscription(
            Image, self._valor('image_topic'), self._on_image, imagenes)
        self._ultima_depuracion = None

        self.add_on_set_parameters_callback(self._al_cambiar_parametros)
        self.get_logger().info('lane_detection iniciado')

    def _valor(self, nombre):
        return self.get_parameter(nombre).value

    def _configuracion(self, cambios=None):
        """Arma la configuración del detector con los parámetros, más los `cambios` dados."""
        cambios = cambios or {}
        valores = {}
        for nombre in CAMPOS_DE_CONFIG:
            valor = cambios.get(nombre, self._valor(nombre))
            valores[nombre] = tuple(valor) if nombre in PARAMETROS_ARREGLOS else valor
        return LaneConfig(**valores)

    def _al_cambiar_parametros(self, parametros):
        """Valida un cambio de parámetros y, si es válido, rehace el detector."""
        cambios = {parametro.name: parametro.value for parametro in parametros}

        fijos = sorted(nombre for nombre in cambios if nombre in PARAMETROS_SOLO_AL_ARRANCAR)
        if fijos:
            return SetParametersResult(
                successful=False,
                reason='%s solo se puede cambiar al arrancar el nodo' % ', '.join(fijos))
        if 'debug_max_fps' in cambios and not cambios['debug_max_fps'] > 0.0:
            return SetParametersResult(
                successful=False, reason='debug_max_fps debe ser mayor que 0')

        de_la_vision = sorted(nombre for nombre in cambios if nombre in CAMPOS_DE_CONFIG)
        if de_la_vision:
            try:
                configuracion = self._configuracion(cambios)
                detector = LaneDetector(configuracion)
            except ValueError as error:
                return SetParametersResult(successful=False, reason=str(error))
            self._detector = detector
            self.get_logger().info(
                'detector actualizado: %s' % ', '.join(de_la_vision), throttle_duration_sec=1.0)
        return SetParametersResult(successful=True)

    def _toca_depurar(self):
        """Indica si hay que dibujar las imágenes de depuración para esta imagen."""
        if not self._valor('publish_debug'):
            return False
        if (self._publicador_camara.get_subscription_count() == 0
                and self._publicador_cenital.get_subscription_count() == 0):
            return False
        ahora = self.get_clock().now()
        intervalo = Duration(seconds=1.0 / self._valor('debug_max_fps'))
        if self._ultima_depuracion is not None and ahora - self._ultima_depuracion < intervalo:
            return False
        self._ultima_depuracion = ahora
        return True

    def _on_image(self, msg: Image):
        try:
            bgr = imagen_a_bgr(msg.height, msg.width, msg.encoding, msg.step, msg.data)
        except ValueError as error:
            self.get_logger().warning('imagen descartada: %s' % error, throttle_duration_sec=5.0)
            return

        detector = self._detector
        depurar = self._toca_depurar()
        try:
            if depurar:
                analisis = detector.analizar(bgr)
                resultado = analisis.resultado
            else:
                resultado = detector.detect(bgr)
        except (cv2.error, np.linalg.LinAlgError) as error:
            self.get_logger().error(
                'imagen descartada: falló la visión: %s' % error, throttle_duration_sec=5.0)
            return

        estado = LaneState()
        estado.header = msg.header
        estado.detected = resultado.detected
        estado.lateral_error_m = resultado.lateral_error_m
        estado.heading_error_rad = resultado.heading_error_rad
        estado.confidence = resultado.confidence
        estado.intersection_detected = resultado.intersection_detected
        self._publisher.publish(estado)

        if depurar:
            self._publicar_depuracion(detector, bgr, analisis, msg.header)

    def _publicar_depuracion(self, detector, bgr, analisis, encabezado):
        """Dibuja y publica las imágenes de depuración. Si falla, la detección sigue."""
        try:
            camara = dibujar_camara(detector, bgr, analisis)
            cenital = dibujar_cenital(detector, analisis)
        except (cv2.error, ValueError, np.linalg.LinAlgError) as error:
            self.get_logger().warning(
                'no se pudo dibujar la depuración: %s' % error, throttle_duration_sec=5.0)
            return
        self._publicador_camara.publish(self._a_mensaje(camara, encabezado))
        self._publicador_cenital.publish(self._a_mensaje(cenital, encabezado))

    @staticmethod
    def _a_mensaje(imagen, encabezado):
        mensaje = Image()
        mensaje.header = encabezado
        mensaje.height, mensaje.width = imagen.shape[:2]
        mensaje.encoding = 'bgr8'
        mensaje.is_bigendian = 0
        mensaje.step = mensaje.width * 3
        mensaje.data = np.ascontiguousarray(imagen).tobytes()
        return mensaje


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
