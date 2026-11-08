"""Nodo ROS2 de bfmc_control: conecta el estado del carril con la lógica del PID."""
from ackermann_msgs.msg import AckermannDriveStamped
from bfmc_control.control_logic import (
    ControlConfig, ControlLogic, LaneInput,
)
from bfmc_control.pure_pursuit import PurePursuitConfig
from bfmc_interfaces.msg import LaneState
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy

PARAMETROS_DE_LA_LOGICA = (
    'kp', 'ki', 'kd', 'heading_weight_m_per_rad', 'deadband_m', 'integral_limit_rad',
    'max_steer_rad', 'max_steer_rate_rad_s', 'filter_alpha', 'min_confidence',
    'cruise_speed_mps', 'lost_speed_factor', 'slowdown_gain', 'max_dt_s', 'hold_steer_s',
)
# Pure Pursuit (A2-155): nombre del parámetro = campo de PurePursuitConfig.
PARAMETROS_DE_PURE_PURSUIT = (
    'wheelbase_m', 'camera_to_rear_axle_m', 'lookahead_min_m', 'lookahead_max_m',
    'lookahead_gain_s', 'max_lateral_accel_mps2', 'speed_min_factor',
)


class ControlNode(Node):
    """Lee /perception/lane y publica el comando de dirección en /control/lane_cmd.

    `algorithm` elige el PID o Pure Pursuit (se puede cambiar en vivo). El encabezado del
    comando lleva el instante de la imagen de la que salió (el de LaneState), así se
    puede medir la latencia de punta a punta en /cmd/drive.

    Todos los parámetros se declaran sin valor por defecto: los valores viven en
    config/params.yaml y, si falta alguno, el nodo falla al arrancar con un
    mensaje claro.
    """

    def __init__(self):
        super().__init__('control')

        self.declare_parameter('lane_topic', Parameter.Type.STRING)
        self.declare_parameter('cmd_topic', Parameter.Type.STRING)
        self.declare_parameter('frame_id', Parameter.Type.STRING)
        for nombre in PARAMETROS_DE_LA_LOGICA + PARAMETROS_DE_PURE_PURSUIT:
            self.declare_parameter(nombre, Parameter.Type.DOUBLE)
        self.declare_parameter('algorithm', Parameter.Type.STRING)

        self._frame_id = self.get_parameter('frame_id').value
        self._logic = ControlLogic(self._configuracion())

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._publisher = self.create_publisher(
            AckermannDriveStamped, self.get_parameter('cmd_topic').value, qos)
        self._subscription = self.create_subscription(
            LaneState, self.get_parameter('lane_topic').value, self._on_lane, qos)

        self.add_on_set_parameters_callback(self._al_cambiar_parametros)
        self.get_logger().info('control iniciado')

    def _configuracion(self, cambios=None):
        """Arma la configuración con los parámetros, más los `cambios` dados."""
        cambios = cambios or {}

        def valor(nombre):
            return cambios.get(nombre, self.get_parameter(nombre).value)

        pure_pursuit = PurePursuitConfig(
            **{nombre: valor(nombre) for nombre in PARAMETROS_DE_PURE_PURSUIT})
        return ControlConfig(
            **{nombre: valor(nombre) for nombre in PARAMETROS_DE_LA_LOGICA},
            algorithm=valor('algorithm'), pure_pursuit=pure_pursuit)

    def _al_cambiar_parametros(self, parametros):
        """Valida un cambio de parámetros de la lógica y, si es válido, lo aplica en vivo."""
        propios = PARAMETROS_DE_LA_LOGICA + PARAMETROS_DE_PURE_PURSUIT + ('algorithm',)
        cambios = {p.name: p.value for p in parametros if p.name in propios}
        if not cambios:
            return SetParametersResult(successful=True)
        try:
            configuracion = self._configuracion(cambios)
        except (TypeError, ValueError) as error:
            return SetParametersResult(successful=False, reason=str(error))
        self._logic.set_config(configuracion)
        return SetParametersResult(successful=True)

    def _on_lane(self, msg: LaneState):
        now = self.get_clock().now()
        salida = self._logic.update(
            LaneInput(
                detected=msg.detected,
                lateral_error_m=msg.lateral_error_m,
                heading_error_rad=msg.heading_error_rad,
                confidence=msg.confidence,
                curvature=msg.curvature,
                reference_distance_m=msg.reference_distance_m,
            ),
            now.nanoseconds / 1e9)

        comando = AckermannDriveStamped()
        # El instante de la imagen, no el de ahora: así /cmd/drive dice de cuándo es el dato
        # con el que se decidió y `ros2 topic delay` mide la latencia de punta a punta.
        comando.header.stamp = msg.header.stamp
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
