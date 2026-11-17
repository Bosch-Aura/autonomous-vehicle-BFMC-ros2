"""Nodo ROS2 de bfmc_sign_detection: detecta señales y estima su distancia."""
from bfmc_interfaces.msg import SignDetection, SignDetectionArray
from bfmc_sign_detection.image_utils import imagen_a_bgr, profundidad_a_metros
from bfmc_sign_detection.sign_detection_logic import (
    parse_class_map, SignConfig, SignProcessor,
)
from bfmc_sign_detection.yolo_detector import crear_detector_yolo
import rclpy
from rcl_interfaces.msg import SetParametersResult
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

    `enabled` lo prende y lo apaga con el nodo andando (por ejemplo, desde el visor).
    Apagado, el nodo no carga el modelo, no se suscribe a las imágenes y no publica nada:
    no gasta CPU ni GPU, y `bfmc_decision` ignora las señales. Al prenderlo carga el modelo
    (puede tardar unos segundos); si falta Ultralytics o el modelo, el cambio se rechaza con
    el motivo y el nodo sigue apagado. Una vez cargado, el modelo queda en memoria: volver a
    prender es instantáneo. Si arranca con `enabled: true`, un modelo que falta es un error
    de arranque, como siempre.

    Args:
        detector_factory: función (model_path, device, confidence) que devuelve un
            detector. Por defecto crea el de YOLO; los tests inyectan uno falso.
        parameter_overrides: valores de parámetros que pisan a los del YAML (para los tests).
    """

    def __init__(self, detector_factory=crear_detector_yolo, parameter_overrides=None):
        super().__init__('sign_detection', parameter_overrides=parameter_overrides)

        for nombre in PARAMETROS_TEXTO:
            self.declare_parameter(nombre, Parameter.Type.STRING)
        for nombre in PARAMETROS_DECIMALES:
            self.declare_parameter(nombre, Parameter.Type.DOUBLE)
        self.declare_parameter('min_depth_pixels', Parameter.Type.INTEGER)
        self.declare_parameter('class_map', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('enabled', Parameter.Type.BOOL)

        self._detector_factory = detector_factory
        self._procesador = None
        self._sub_color = None
        self._sub_profundidad = None
        self._profundidad = None
        self._profundidad_instante = None

        resultados = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._publisher = self.create_publisher(
            SignDetectionArray, self._valor('signs_topic'), resultados)

        if self._valor('enabled'):
            self._activar()
            self.get_logger().info('sign_detection iniciado (activo)')
        else:
            self.get_logger().info(
                'sign_detection iniciado, apagado: se prende con el parámetro enabled')
        self.add_on_set_parameters_callback(self._al_cambiar_parametros)

    def _activar(self):
        """Carga el modelo (la primera vez) y empieza a leer las imágenes.

        Lanza la excepción de la fábrica si no se puede cargar: quien llama decide qué hacer.
        """
        if self._procesador is None:
            detector = self._detector_factory(
                self._valor('model_path'), self._valor('device'),
                self._valor('yolo_confidence'))
            self._procesador = SignProcessor(detector, SignConfig(
                class_map=parse_class_map(self._valor('class_map')),
                min_confidence=self._valor('min_confidence'),
                depth_window_fraction=self._valor('depth_window_fraction'),
                min_depth_m=self._valor('min_depth_m'),
                max_depth_m=self._valor('max_depth_m'),
                min_depth_pixels=self._valor('min_depth_pixels'),
            ))
        if self._sub_color is not None:
            return
        imagenes = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._sub_color = self.create_subscription(
            Image, self._valor('color_topic'), self._on_color, imagenes)
        self._sub_profundidad = self.create_subscription(
            Image, self._valor('depth_topic'), self._on_profundidad, imagenes)

    def _desactivar(self):
        """Deja de leer las imágenes. El modelo queda cargado."""
        for suscripcion in (self._sub_color, self._sub_profundidad):
            if suscripcion is not None:
                self.destroy_subscription(suscripcion)
        self._sub_color = None
        self._sub_profundidad = None
        self._profundidad = None
        self._profundidad_instante = None

    def _al_cambiar_parametros(self, parametros):
        """Prende o apaga la detección. Si no se puede prender, se rechaza con el motivo."""
        for parametro in parametros:
            if parametro.name != 'enabled':
                continue
            try:
                if parametro.value:
                    self._activar()
                else:
                    self._desactivar()
            except Exception as error:  # el modelo o Ultralytics pueden fallar de muchas formas
                return SetParametersResult(
                    successful=False, reason='no se pudo activar la detección de señales: %s'
                    % error)
        return SetParametersResult(successful=True)

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
