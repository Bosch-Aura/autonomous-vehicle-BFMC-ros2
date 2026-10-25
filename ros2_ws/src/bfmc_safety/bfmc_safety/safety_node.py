"""Nodo ROS2 de bfmc_safety: conecta el ultrasónico con la lógica de seguridad."""
from bfmc_interfaces.msg import SafetyStatus
from bfmc_safety.safety_logic import Reading, SafetyConfig, SafetyLogic
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Range


class SafetyNode(Node):
    """Lee /hardware/ultrasonic y publica /safety/status a frecuencia fija.

    Publica a frecuencia fija, y no solo cuando llega una lectura, para poder
    informar una emergencia cuando el sensor deja de enviar datos.

    Todos los parámetros se declaran sin valor por defecto: los valores viven en
    config/params.yaml y, si falta alguno, el nodo falla al arrancar.
    """

    def __init__(self):
        super().__init__('safety')

        self.declare_parameter('ultrasonic_topic', Parameter.Type.STRING)
        self.declare_parameter('status_topic', Parameter.Type.STRING)
        self.declare_parameter('frame_id', Parameter.Type.STRING)
        self.declare_parameter('publish_rate_hz', Parameter.Type.DOUBLE)
        self.declare_parameter('stop_distance_m', Parameter.Type.DOUBLE)
        self.declare_parameter('resume_distance_m', Parameter.Type.DOUBLE)
        self.declare_parameter('stale_timeout_s', Parameter.Type.DOUBLE)
        self.declare_parameter('emergency_on_invalid', Parameter.Type.BOOL)

        self._frame_id = self.get_parameter('frame_id').value
        self._logic = SafetyLogic(SafetyConfig(
            stop_distance_m=self.get_parameter('stop_distance_m').value,
            resume_distance_m=self.get_parameter('resume_distance_m').value,
            stale_timeout_s=self.get_parameter('stale_timeout_s').value,
            emergency_on_invalid=self.get_parameter('emergency_on_invalid').value,
        ))

        # Los sensores se leen con calidad "best effort": si el consumidor se atrasa,
        # conviene perder una lectura vieja y no acumular retraso.
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        status_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._publisher = self.create_publisher(
            SafetyStatus, self.get_parameter('status_topic').value, status_qos)
        self._subscription = self.create_subscription(
            Range, self.get_parameter('ultrasonic_topic').value, self._on_range, sensor_qos)
        self._timer = self.create_timer(
            1.0 / self.get_parameter('publish_rate_hz').value, self._publish_status)

        self.get_logger().info('safety iniciado')

    def _now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _on_range(self, msg: Range):
        self._logic.on_reading(
            Reading(range_m=msg.range, min_range_m=msg.min_range, max_range_m=msg.max_range),
            self._now_s())

    def _publish_status(self):
        resultado = self._logic.evaluate(self._now_s())
        estado = SafetyStatus()
        estado.header.stamp = self.get_clock().now().to_msg()
        estado.header.frame_id = self._frame_id
        estado.emergency = resultado.emergency
        estado.reason = int(resultado.reason)
        estado.sensor_valid = resultado.sensor_valid
        estado.distance_m = float(resultado.distance_m)
        self._publisher.publish(estado)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
