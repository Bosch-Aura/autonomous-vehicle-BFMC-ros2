"""Nodo ROS2 de bfmc_hardware_bridge: único dueño del puerto serial."""
from ackermann_msgs.msg import AckermannDriveStamped
from bfmc_hardware_bridge.hardware_bridge_logic import BridgeConfig, BridgeLogic
from bfmc_hardware_bridge.line_buffer import LineBuffer
from bfmc_hardware_bridge.serial_port import PySerialPort
from bfmc_hardware_bridge.units import Esp32Limits
from bfmc_interfaces.msg import HardwareStatus, SafetyStatus
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

PARAMETROS_DECIMALES = (
    'send_rate_hz', 'status_rate_hz', 'reconnect_period_s', 'link_timeout_s',
    'cmd_timeout_s', 'arm_retry_s', 'max_speed_mps', 'max_steer_rad',
)
PARAMETROS_ENTEROS = (
    'baud_rate', 'duty_start', 'duty_max', 'servo_left', 'servo_center', 'servo_right',
)
PARAMETROS_TEXTO = (
    'serial_port', 'esp32_mode', 'cmd_topic', 'safety_topic', 'status_topic',
    'status_frame_id', 'arm_service',
)
MODOS_ESP32 = ('AUTO', 'MANUAL')


def abrir_puerto_real(ruta, baud_rate):
    return PySerialPort(ruta, baud_rate)


class HardwareBridgeNode(Node):
    """Traduce los comandos de ROS al protocolo del ESP32 y publica lo que este reporta.

    En cada ciclo lee el puerto, procesa lo recibido y envía velocidad y
    dirección: las del último comando, o 0 y centro si hay una emergencia o el
    comando es viejo. Reenviar en cada ciclo hace de heartbeat para el watchdog
    del ESP32.

    Todos los parámetros se declaran sin valor por defecto: los valores viven en
    config/params.yaml y, si falta alguno, el nodo falla al arrancar.

    Args:
        port_factory: función (ruta, baud_rate) que devuelve un puerto. Los tests
            inyectan uno simulado.
    """

    def __init__(self, port_factory=abrir_puerto_real):
        super().__init__('hardware_bridge')
        self._port_factory = port_factory

        for nombre in PARAMETROS_TEXTO:
            self.declare_parameter(nombre, Parameter.Type.STRING)
        for nombre in PARAMETROS_ENTEROS:
            self.declare_parameter(nombre, Parameter.Type.INTEGER)
        for nombre in PARAMETROS_DECIMALES:
            self.declare_parameter(nombre, Parameter.Type.DOUBLE)

        modo = self._param('esp32_mode')
        if modo not in MODOS_ESP32:
            raise ValueError('esp32_mode debe ser AUTO o MANUAL, no %r' % modo)
        self._logic = BridgeLogic(BridgeConfig(
            link_timeout_s=self._param('link_timeout_s'),
            cmd_timeout_s=self._param('cmd_timeout_s'),
            arm_retry_s=self._param('arm_retry_s'),
            auto_mode=(modo == 'AUTO'),
            limits=Esp32Limits(
                max_speed_mps=self._param('max_speed_mps'),
                max_steer_rad=self._param('max_steer_rad'),
                duty_start=self._param('duty_start'),
                duty_max=self._param('duty_max'),
                servo_left=self._param('servo_left'),
                servo_center=self._param('servo_center'),
                servo_right=self._param('servo_right'),
            ),
        ))
        self._lineas = LineBuffer()
        self._port = None
        self._ultimo_intento = None
        self._ultimo_estado = None
        self._estado_informado = None

        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._pub_estado = self.create_publisher(
            HardwareStatus, self._param('status_topic'), reliable)
        self.create_subscription(
            AckermannDriveStamped, self._param('cmd_topic'), self._on_cmd, reliable)
        self.create_subscription(
            SafetyStatus, self._param('safety_topic'), self._on_safety, reliable)
        self.create_service(Trigger, self._param('arm_service'), self._on_arm)

        self._timer = self.create_timer(1.0 / self._param('send_rate_hz'), self._ciclo)
        self.get_logger().info('hardware_bridge iniciado')

    # --- parámetros ---------------------------------------------------------
    def _param(self, nombre):
        return self.get_parameter(nombre).value

    def _now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    # --- mensajes y servicios de ROS -----------------------------------------
    def _on_cmd(self, msg: AckermannDriveStamped):
        self._logic.on_drive_command(msg.drive.speed, msg.drive.steering_angle, self._now_s())

    def _on_safety(self, msg: SafetyStatus):
        self._logic.on_safety(msg.emergency)

    def _on_arm(self, _request, response):
        # Es la salida de FAULT: el puente nunca rearma solo después de un fallo.
        self._logic.request_arm()
        response.success = True
        response.message = 'armado pedido; se confirma en %s' % self._param('status_topic')
        self.get_logger().info('armado del ESP32 pedido por servicio')
        return response

    # --- puerto serial ------------------------------------------------------
    def _asegurar_puerto(self, now):
        if self._port is not None:
            return
        espera = self._param('reconnect_period_s')
        if self._ultimo_intento is not None and (now - self._ultimo_intento) < espera:
            return
        self._ultimo_intento = now
        try:
            self._port = self._port_factory(self._param('serial_port'), self._param('baud_rate'))
        except (OSError, ValueError) as error:
            self._port = None
            self.get_logger().warning('no se pudo abrir el puerto serial: %s' % error)
            return
        self._lineas = LineBuffer()
        self._logic.on_port_opened()
        self.get_logger().info('puerto serial abierto: %s' % self._param('serial_port'))

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
                    self._logic.on_serial_line(linea, now)
                self._port.write(self._logic.next_frames(now))
            except OSError as error:
                self._cerrar_puerto(error)

        self._informar_cambio_de_estado(now)
        self._publicar_estado(now)

    # --- publicaciones --------------------------------------------------------
    def _informar_cambio_de_estado(self, now):
        estado = self._logic.state(now)
        clave = (estado.esp32_state, estado.esp32_mode)
        if clave == self._estado_informado:
            return
        self._estado_informado = clave
        texto = 'ESP32 en estado %s, modo %s' % (
            estado.esp32_state or '?', estado.esp32_mode or '?')
        if estado.fault:
            self.get_logger().error(
                texto + ': frenado; para rearmarlo, llamar a %s' % self._param('arm_service'))
        else:
            self.get_logger().info(texto)

    def _publicar_estado(self, now):
        periodo = 1.0 / self._param('status_rate_hz')
        if self._ultimo_estado is not None and (now - self._ultimo_estado) < periodo:
            return
        self._ultimo_estado = now
        estado = self._logic.state(now)
        msg = HardwareStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._param('status_frame_id')
        msg.link_up = estado.link_up
        # El firmware no distingue el motivo del FAULT (watchdog, E-STOP u
        # obstáculo); en todos los casos frenó y espera que lo rearmen.
        msg.watchdog_tripped = estado.fault
        # El firmware no informa su versión ni confirma secuencias (ADR-0002).
        msg.firmware_version = ''
        msg.last_seq = 0
        self._pub_estado.publish(msg)

    def destroy_node(self):
        if self._port is not None:
            try:
                self._port.write(self._logic.shutdown_frames())
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
