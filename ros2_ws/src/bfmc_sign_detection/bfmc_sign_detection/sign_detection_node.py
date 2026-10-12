"""Nodo ROS2 de bfmc_sign_detection: detecta señales y estima su distancia."""
from bfmc_interfaces.msg import SignDetection, SignDetectionArray
from bfmc_sign_detection.image_utils import imagen_a_bgr, profundidad_a_metros
from bfmc_sign_detection.sign_detection_logic import (
    parse_class_map, SignConfig, SignProcessor,
)
from bfmc_sign_detection.yolo_detector import crear_detector_yolo
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

PARAMETROS_TEXTO = ('color_topic', 'depth_topic', 'signs_topic', 'model_path', 'device')
PARAMETROS_DECIMALES = (
    'yolo_confidence', 'min_confidence', 'depth_window_fraction', 'min_depth_m',
    'max_depth_m', 'depth_unit_m', 'max_depth_age_s',
)


class SignDetectionNode(Node):
    """Lee color y profundidad alineada y publica las señales detectadas.

    Publica un mensaje por cada imagen de color, también cuando no hay señales:
    así quien lo consume sabe que el detector sigue vivo. Usa la última profundidad
    recibida si no es más vieja que `max_depth_age_s`; si no, la distancia queda
    como NaN.

    Todos los parámetros se declaran sin valor por defecto: los valores viven en
    config/params.yaml y, si falta alguno, el nodo falla al arrancar.

    Args:
        detector_factory: función (model_path, device, confidence) que devuelve un
            detector. Por defecto crea el de YOLO; los tests inyectan uno falso.
    """

    def __init__(self, detector_factory=crear_detector_yolo):
        super().__init__('sign_detection')

        for nombre in PARAMETROS_TEXTO:
            self.declare_parameter(nombre, Parameter.Type.STRING)
        for nombre in PARAMETROS_DECIMALES:
            self.declare_parameter(nombre, Parameter.Type.DOUBLE)
        self.declare_parameter('min_depth_pixels', Parameter.Type.INTEGER)
        self.declare_parameter('class_map', Parameter.Type.STRING_ARRAY)

        detector = detector_factory(
            self._valor('model_path'), self._valor('device'), self._valor('yolo_confidence'))
        self._procesador = SignProcessor(detector, SignConfig(
            class_map=parse_class_map(self._valor('class_map')),
            min_confidence=self._valor('min_confidence'),
            depth_window_fraction=self._valor('depth_window_fraction'),
            min_depth_m=self._valor('min_depth_m'),
            max_depth_m=self._valor('max_depth_m'),
            min_depth_pixels=self._valor('min_depth_pixels'),
        ))

        self._profundidad = None
        self._profundidad_instante = None

        imagenes = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        resultados = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._publisher = self.create_publisher(
            SignDetectionArray, self._valor('signs_topic'), resultados)
        self._sub_color = self.create_subscription(
            Image, self._valor('color_topic'), self._on_color, imagenes)
        self._sub_profundidad = self.create_subscription(
            Image, self._valor('depth_topic'), self._on_profundidad, imagenes)

        self.get_logger().info('sign_detection iniciado')

    def _valor(self, nombre):
        return self.get_parameter(nombre).value

    def _now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _on_profundidad(self, msg: Image):
        try:
            self._profundidad = profundidad_a_metros(
                msg.height, msg.width, msg.encoding, msg.step, msg.is_bigendian, msg.data,
                self._valor('depth_unit_m'))
        except ValueError as error:
            self.get_logger().warning(
                'profundidad descartada: %s' % error, throttle_duration_sec=5.0)
            return
        self._profundidad_instante = self._now_s()

    def _profundidad_vigente(self):
        if self._profundidad is None:
            return None
        if (self._now_s() - self._profundidad_instante) > self._valor('max_depth_age_s'):
            return None
        return self._profundidad

    def _on_color(self, msg: Image):
        try:
            bgr = imagen_a_bgr(msg.height, msg.width, msg.encoding, msg.step, msg.data)
        except ValueError as error:
            self.get_logger().warning('imagen descartada: %s' % error, throttle_duration_sec=5.0)
            return

        senales = self._procesador.process(bgr, self._profundidad_vigente())

        salida = SignDetectionArray()
        salida.header = msg.header
        for senal in senales:
            deteccion = SignDetection()
            deteccion.label = senal.label
            deteccion.confidence = senal.confidence
            deteccion.distance_m = senal.distance_m
            deteccion.x_min, deteccion.y_min = senal.x_min, senal.y_min
            deteccion.x_max, deteccion.y_max = senal.x_max, senal.y_max
            salida.detections.append(deteccion)
        self._publisher.publish(salida)


def main(args=None):
    rclpy.init(args=args)
    node = SignDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
