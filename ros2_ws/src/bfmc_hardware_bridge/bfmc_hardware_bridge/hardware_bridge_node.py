"""Nodo ROS2 de bfmc_hardware_bridge: único dueño del puerto serial."""
from ackermann_msgs.msg import AckermannDriveStamped
from bfmc_interfaces.msg import HardwareStatus, SafetyStatus
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Range

from bfmc_hardware_bridge.hardware_bridge_logic import BridgeConfig, BridgeLogic
from bfmc_hardware_bridge.line_buffer import LineBuffer
from bfmc_hardware_bridge.serial_port import PySerialPort

PARAMETROS_DECIMALES = (
    'send_rate_hz', 'status_rate_hz', 'reconnect_period_s', 'link_timeout_s',
    'cmd_timeout_s', 'max_speed_mps', 'max_steer_rad', 'ultrasonic_min_range_m',
    'ultrasonic_max_range_m', 'ultrasonic_fov_rad',
)
PARAMETROS_TEXTO = (
    'serial_port', 'cmd_topic', 'safety_topic', 'ultrasonic_topic', 'status_topic',
    'ultrasonic_frame_id', 'status_frame_id',
)


def abrir_puerto_real(ruta, baud_rate):
    return PySerialPort(ruta, baud_rate)


class HardwareBridgeNode(Node):
    """Traduce los comandos de ROS al protocolo del ESP32 y publica lo que este reporta.

    En cada ciclo lee el puerto, procesa lo recibido y envía una trama: el último
    comando, o freno si hay una emergencia o el comando es viejo. Reenviar en cada
    ciclo hace de heartbeat para el watchdog del ESP32.

    Todos los parámetros se declaran sin valor por defecto: los valores viven en
    config/params.yaml y, si falta alguno, el nodo falla al arrancar.

    Args:
        port_factory: función (ruta, baud_rate) que devuelve un puerto. Los tests
            inyectan uno simulado.
    """

    def __init__(self, port_factory=abrir_puerto_real):
        super().__init__('hardware_bridge')
        self._port_factory = port_factory

        self.declare_parameter('baud_rate', Parameter.Type.INTEGER)
        for nombre in PARAMETROS_TEXTO:
            self.declare_parameter(nombre, Parameter.Type.STRING)
        for nombre in PARAMETROS_DECIMALES:
            self.declare_parameter(nombre, Parameter.Type.DOUBLE)

        self._logic = BridgeLogic(BridgeConfig(
            link_timeout_s=self._decimal('link_timeout_s'),
            cmd_timeout_s=self._decimal('cmd_timeout_s'),
            max_speed_mps=self._decimal('max_speed_mps'),
            max_steer_rad=self._decimal('max_steer_rad'),
            ultrasonic_min_range_m=self._decimal('ultrasonic_min_range_m'),
            ultrasonic_max_range_m=self._decimal('ultrasonic_max_range_m'),
        ))
        self._lineas = LineBuffer()
        self._port = None
        self._ultimo_intento = None
        self._ultimo_estado = None

        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        sensor = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._pub_ultrasonico = self.create_publisher(
            Range, self._texto('ultrasonic_topic'), sensor)
        self._pub_estado = self.create_publisher(
            HardwareStatus, self._texto('status_topic'), reliable)
        self.create_subscription(
            AckermannDriveStamped, self._texto('cmd_topic'), self._on_cmd, reliable)
        self.create_subscription(
            SafetyStatus, self._texto('safety_topic'), self._on_safety, reliable)

        self._timer = self.create_timer(1.0 / self._decimal('send_rate_hz'), self._ciclo)
        self.get_logger().info('hardware_bridge iniciado')

    # --- parámetros ---------------------------------------------------------
    def _decimal(self, nombre):
        return self.get_parameter(nombre).value

    def _texto(self, nombre):
        return self.get_parameter(nombre).value

    def _now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    # --- mensajes de ROS ----------------------------------------------------
    def _on_cmd(self, msg: AckermannDriveStamped):
        self._logic.on_drive_command(msg.drive.speed, msg.drive.steering_angle, self._now_s())

    def _on_safety(self, msg: SafetyStatus):
        self._logic.on_safety(msg.emergency)

    # --- puerto serial ------------------------------------------------------
    def _asegurar_puerto(self, now):
        if self._port is not None:
            return
        espera = self._decimal('reconnect_period_s')
        if self._ultimo_intento is not None and (now - self._ultimo_intento) < espera:
            return
        self._ultimo_intento = now
        try:
            self._port = self._port_factory(
                self._texto('serial_port'), int(self.get_parameter('baud_rate').value))
            self.get_logger().info('puerto serial abierto: %s' % self._texto('serial_port'))
        except (OSError, ValueError) as error:
            self._port = None
            self.get_logger().warning('no se pudo abrir el puerto serial: %s' % error)

    def _cerrar_puerto(self, motivo):
        self.get_logger().warning('puerto serial caído: %s' % motivo)
        try:
            self._port.close()
        except OSError:
            pass
        self._port = None

    def _ciclo(self):
        now = self._now_s()
        self._asegurar_puerto(now)

        if self._port is not None:
            try:
                for linea in self._lineas.feed(self._port.read_available()):
                    distancia = self._logic.on_serial_line(linea, now)
                    if distancia is not None:
                        self._publicar_ultrasonico(distancia)
                self._port.write(self._logic.next_frame(now))
            except OSError as error:
                self._cerrar_puerto(error)

        self._publicar_estado(now)

    # --- publicaciones --------------------------------------------------------
    def _publicar_ultrasonico(self, distancia_m):
        lectura = Range()
        lectura.header.stamp = self.get_clock().now().to_msg()
        lectura.header.frame_id = self._texto('ultrasonic_frame_id')
        lectura.radiation_type = Range.ULTRASOUND
        lectura.field_of_view = self._decimal('ultrasonic_fov_rad')
        lectura.min_range = self._decimal('ultrasonic_min_range_m')
        lectura.max_range = self._decimal('ultrasonic_max_range_m')
        lectura.range = distancia_m
        self._pub_ultrasonico.publish(lectura)

    def _publicar_estado(self, now):
        periodo = 1.0 / self._decimal('status_rate_hz')
        if self._ultimo_estado is not None and (now - self._ultimo_estado) < periodo:
            return
        self._ultimo_estado = now
        estado = self._logic.state(now)
        msg = HardwareStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._texto('status_frame_id')
        msg.link_up = estado.link_up
        msg.watchdog_tripped = estado.watchdog_tripped
        msg.firmware_version = estado.firmware_version
        msg.last_seq = estado.last_seq
        self._pub_estado.publish(msg)

    def destroy_node(self):
        if self._port is not None:
            try:
                self._port.close()
            except OSError:
                pass
            self._port = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HardwareBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
