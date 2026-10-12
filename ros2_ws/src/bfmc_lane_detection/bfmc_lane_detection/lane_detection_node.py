"""Nodo ROS2 de bfmc_lane_detection: convierte las imágenes de la cámara en LaneState."""
from bfmc_interfaces.msg import LaneState
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from bfmc_lane_detection.image_utils import imagen_a_bgr
from bfmc_lane_detection.lane_detection_logic import LaneConfig, LaneDetector

PARAMETROS_TEXTO = ('image_topic', 'lane_topic')
PARAMETROS_ENTEROS = (
    'morph_kernel_px', 'warp_width', 'warp_height', 'n_windows', 'window_margin_px',
    'min_pixels', 'full_confidence_pixels', 'intersection_min_rows',
)
PARAMETROS_DECIMALES = (
    'roi_top_fraction', 'lane_width_m', 'lane_width_px', 'forward_range_m',
    'reference_row_fraction', 'intersection_band_top_fraction', 'intersection_row_fraction',
)


class LaneDetectionNode(Node):
    """Lee las imágenes de color y publica el estado del carril en /perception/lane.

    Cada imagen se procesa sola. El encabezado del resultado es el de la imagen, así
    quien lo consume sabe de qué instante es la medición.

    Todos los parámetros se declaran sin valor por defecto: los valores viven en
    config/params.yaml y, si falta alguno, el nodo falla al arrancar.
    """

    def __init__(self):
        super().__init__('lane_detection')

        for nombre in PARAMETROS_TEXTO:
            self.declare_parameter(nombre, Parameter.Type.STRING)
        for nombre in PARAMETROS_ENTEROS:
            self.declare_parameter(nombre, Parameter.Type.INTEGER)
        for nombre in PARAMETROS_DECIMALES:
            self.declare_parameter(nombre, Parameter.Type.DOUBLE)
        self.declare_parameter('white_hsv_min', Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter('white_hsv_max', Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter('warp_src', Parameter.Type.DOUBLE_ARRAY)

        self._detector = LaneDetector(LaneConfig(
            white_hsv_min=tuple(self._valor('white_hsv_min')),
            white_hsv_max=tuple(self._valor('white_hsv_max')),
            morph_kernel_px=self._valor('morph_kernel_px'),
            roi_top_fraction=self._valor('roi_top_fraction'),
            warp_src=tuple(self._valor('warp_src')),
            warp_width=self._valor('warp_width'),
            warp_height=self._valor('warp_height'),
            lane_width_m=self._valor('lane_width_m'),
            lane_width_px=self._valor('lane_width_px'),
            forward_range_m=self._valor('forward_range_m'),
            n_windows=self._valor('n_windows'),
            window_margin_px=self._valor('window_margin_px'),
            min_pixels=self._valor('min_pixels'),
            reference_row_fraction=self._valor('reference_row_fraction'),
            full_confidence_pixels=self._valor('full_confidence_pixels'),
            intersection_band_top_fraction=self._valor('intersection_band_top_fraction'),
            intersection_row_fraction=self._valor('intersection_row_fraction'),
            intersection_min_rows=self._valor('intersection_min_rows'),
        ))

        # Las imágenes se leen con calidad "best effort" y cola de 1: si el nodo se
        # atrasa, conviene procesar la imagen más nueva y no acumular retraso.
        imagenes = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        resultados = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._publisher = self.create_publisher(
            LaneState, self._valor('lane_topic'), resultados)
        self._subscription = self.create_subscription(
            Image, self._valor('image_topic'), self._on_image, imagenes)

        self.get_logger().info('lane_detection iniciado')

    def _valor(self, nombre):
        return self.get_parameter(nombre).value

    def _on_image(self, msg: Image):
        try:
            bgr = imagen_a_bgr(msg.height, msg.width, msg.encoding, msg.step, msg.data)
        except ValueError as error:
            self.get_logger().warning('imagen descartada: %s' % error, throttle_duration_sec=5.0)
            return

        resultado = self._detector.detect(bgr)
        estado = LaneState()
        estado.header = msg.header
        estado.detected = resultado.detected
        estado.lateral_error_m = resultado.lateral_error_m
        estado.heading_error_rad = resultado.heading_error_rad
        estado.confidence = resultado.confidence
        estado.intersection_detected = resultado.intersection_detected
        self._publisher.publish(estado)


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
