"""Nodo ROS2 de bfmc_control: conecta el estado del carril con la lógica del PID."""
from ackermann_msgs.msg import AckermannDriveStamped
from bfmc_interfaces.msg import LaneState
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy

from bfmc_control.control_logic import (
    ControlConfig, ControlLogic, LaneInput,
)

PARAMETROS_DE_LA_LOGICA = (
    'kp', 'ki', 'kd', 'heading_weight_m_per_rad', 'deadband_m', 'integral_limit_rad',
    'max_steer_rad', 'max_steer_rate_rad_s', 'filter_alpha', 'min_confidence',
    'cruise_speed_mps', 'lost_speed_factor', 'slowdown_gain', 'max_dt_s',
)


class ControlNode(Node):
    """Lee /perception/lane y publica el comando de dirección en /control/lane_cmd.

    Todos los parámetros se declaran sin valor por defecto: los valores viven en
    config/params.yaml y, si falta alguno, el nodo falla al arrancar con un
    mensaje claro.
    """

    def __init__(self):
        super().__init__('control')

        self.declare_parameter('lane_topic', Parameter.Type.STRING)
        self.declare_parameter('cmd_topic', Parameter.Type.STRING)
        self.declare_parameter('frame_id', Parameter.Type.STRING)
        for nombre in PARAMETROS_DE_LA_LOGICA:
            self.declare_parameter(nombre, Parameter.Type.DOUBLE)

        self._frame_id = self.get_parameter('frame_id').value
        self._logic = ControlLogic(ControlConfig(**{
            nombre: self.get_parameter(nombre).value for nombre in PARAMETROS_DE_LA_LOGICA}))

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._publisher = self.create_publisher(
            AckermannDriveStamped, self.get_parameter('cmd_topic').value, qos)
        self._subscription = self.create_subscription(
            LaneState, self.get_parameter('lane_topic').value, self._on_lane, qos)

        self.get_logger().info('control iniciado')

    def _on_lane(self, msg: LaneState):
        now = self.get_clock().now()
        salida = self._logic.update(
            LaneInput(
                detected=msg.detected,
                lateral_error_m=msg.lateral_error_m,
                heading_error_rad=msg.heading_error_rad,
                confidence=msg.confidence,
            ),
            now.nanoseconds / 1e9)

        comando = AckermannDriveStamped()
        comando.header.stamp = now.to_msg()
        comando.header.frame_id = self._frame_id
        comando.drive.steering_angle = salida.steering_rad
        comando.drive.speed = salida.speed_mps
        self._publisher.publish(comando)


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
