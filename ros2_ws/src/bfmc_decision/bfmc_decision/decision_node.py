"""Nodo ROS2 de bfmc_decision: único nodo que decide y único publicador de /cmd/drive."""
from ackermann_msgs.msg import AckermannDriveStamped
from bfmc_decision.decision_logic import (
    DecisionConfig, DecisionLogic, Drive, HardwareObservation, LaneObservation, SignObservation,
)
from bfmc_interfaces.action import ManeuverExecute
from bfmc_interfaces.msg import HardwareStatus, LaneState, SignDetectionArray, VehicleState
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy

PARAMETROS_TEXTO = (
    'lane_topic', 'signs_topic', 'hardware_topic', 'lane_cmd_topic',
    'maneuver_cmd_topic', 'drive_topic', 'state_topic', 'maneuver_action_name',
    'frame_id', 'intersection_direction',
)
PARAMETROS_DECIMALES = (
    'rate_hz', 'hardware_timeout_s', 'lane_cmd_timeout_s',
    'maneuver_cmd_timeout_s', 'signs_timeout_s', 'lane_lost_timeout_s',
    'maneuver_request_timeout_s', 'maneuver_timeout_s', 'maneuver_cooldown_s',
    'min_sign_confidence', 'stop_distance_m', 'intersection_distance_m',
)
PARAMETROS_LISTA = ('stop_labels', 'intersection_labels')


class DecisionNode(Node):
    """Lee las entradas del sistema, decide y publica el comando final.

    En cada ciclo elige una sola fuente para `/cmd/drive` (emergencia, maniobra,
    evento o carril) y publica el estado en `/vehicle/state`. Las maniobras se
    piden por la acción `ManeuverExecute`.

    Todos los parámetros se declaran sin valor por defecto: los valores viven en
    config/params.yaml y, si falta alguno, el nodo falla al arrancar.
    """

    def __init__(self):
        super().__init__('decision')

        for nombre in PARAMETROS_TEXTO:
            self.declare_parameter(nombre, Parameter.Type.STRING)
        for nombre in PARAMETROS_DECIMALES:
            self.declare_parameter(nombre, Parameter.Type.DOUBLE)
        for nombre in PARAMETROS_LISTA:
            self.declare_parameter(nombre, Parameter.Type.STRING_ARRAY)
        self.declare_parameter('intersection_min_frames', Parameter.Type.INTEGER)

        self._logic = DecisionLogic(DecisionConfig(
            hardware_timeout_s=self._valor('hardware_timeout_s'),
            lane_cmd_timeout_s=self._valor('lane_cmd_timeout_s'),
            maneuver_cmd_timeout_s=self._valor('maneuver_cmd_timeout_s'),
            signs_timeout_s=self._valor('signs_timeout_s'),
            lane_lost_timeout_s=self._valor('lane_lost_timeout_s'),
            maneuver_request_timeout_s=self._valor('maneuver_request_timeout_s'),
            maneuver_timeout_s=self._valor('maneuver_timeout_s'),
            maneuver_cooldown_s=self._valor('maneuver_cooldown_s'),
            min_sign_confidence=self._valor('min_sign_confidence'),
            stop_distance_m=self._valor('stop_distance_m'),
            intersection_distance_m=self._valor('intersection_distance_m'),
            stop_labels=tuple(self._valor('stop_labels')),
            intersection_labels=tuple(self._valor('intersection_labels')),
            intersection_direction=self._valor('intersection_direction'),
            intersection_min_frames=int(self._valor('intersection_min_frames')),
        ))

        # Cada pedido de maniobra tiene un número de generación. Si se cancela
        # antes de que el servidor responda, su respuesta tardía no debe dejar una
        # maniobra corriendo sin que la lógica lo sepa.
        self._generacion = 0
        self._generacion_vigente = None
        self._goal_handle = None

        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(LaneState, self._valor('lane_topic'), self._on_lane, reliable)
        self.create_subscription(
            SignDetectionArray, self._valor('signs_topic'), self._on_signs, reliable)
        self.create_subscription(
            HardwareStatus, self._valor('hardware_topic'), self._on_hardware, reliable)
        self.create_subscription(
            AckermannDriveStamped, self._valor('lane_cmd_topic'), self._on_lane_cmd, reliable)
        self.create_subscription(
            AckermannDriveStamped, self._valor('maneuver_cmd_topic'),
            self._on_maneuver_cmd, reliable)

        self._pub_drive = self.create_publisher(
            AckermannDriveStamped, self._valor('drive_topic'), reliable)
        self._pub_state = self.create_publisher(
            VehicleState, self._valor('state_topic'), reliable)
        self._cliente = ActionClient(
            self, ManeuverExecute, self._valor('maneuver_action_name'))

        self._timer = self.create_timer(1.0 / self._valor('rate_hz'), self._ciclo)
        self.get_logger().info('decision iniciado')

    def _valor(self, nombre):
        return self.get_parameter(nombre).value

    def _now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    # --- entradas -----------------------------------------------------------------
    def _on_lane(self, msg: LaneState):
        self._logic.on_lane(
            LaneObservation(msg.detected, msg.intersection_detected), self._now_s())

    def _on_signs(self, msg: SignDetectionArray):
        senales = [SignObservation(s.label, s.confidence, s.distance_m) for s in msg.detections]
        self._logic.on_signs(senales, self._now_s())

    def _on_hardware(self, msg: HardwareStatus):
        self._logic.on_hardware(
            HardwareObservation(msg.link_up, msg.watchdog_tripped, msg.obstacle_stop,
                                msg.obstacle_distance_m),
            self._now_s())

    def _on_lane_cmd(self, msg: AckermannDriveStamped):
        self._logic.on_lane_command(
            Drive(msg.drive.speed, msg.drive.steering_angle), self._now_s())

    def _on_maneuver_cmd(self, msg: AckermannDriveStamped):
        self._logic.on_maneuver_command(
            Drive(msg.drive.speed, msg.drive.steering_angle), self._now_s())

    # --- ciclo de decisión ----------------------------------------------------------
    def _ciclo(self):
        decision = self._logic.step(self._now_s())

        if decision.cancel_maneuver:
            self._cancelar_maniobra()
        if decision.maneuver_request is not None:
            self._pedir_maniobra(decision.maneuver_request)

        self._publicar_comando(decision.command)
        self._publicar_estado(decision)

    def _publicar_comando(self, comando: Drive):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._valor('frame_id')
        msg.drive.speed = comando.speed_mps
        msg.drive.steering_angle = comando.steering_rad
        self._pub_drive.publish(msg)

    def _publicar_estado(self, decision):
        msg = VehicleState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._valor('frame_id')
        msg.state = int(decision.state)
        msg.active_maneuver = decision.active_maneuver
        msg.reason = decision.reason
        self._pub_state.publish(msg)

    # --- maniobras ------------------------------------------------------------------
    def _pedir_maniobra(self, pedido):
        self._generacion += 1
        generacion = self._generacion
        self._generacion_vigente = generacion

        if not self._cliente.server_is_ready():
            self.get_logger().warning('el servidor de maniobras no está disponible')
            self._generacion_vigente = None
            self._logic.on_maneuver_goal_response(False, self._now_s())
            return

        meta = ManeuverExecute.Goal()
        meta.maneuver = pedido.name
        meta.direction = pedido.direction
        futuro = self._cliente.send_goal_async(meta)
        futuro.add_done_callback(lambda f: self._on_respuesta(generacion, f))

    def _on_respuesta(self, generacion, futuro):
        goal_handle = futuro.result()
        if generacion != self._generacion_vigente:
            # Se canceló antes de que el servidor respondiera: si lo aceptó, se cancela.
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return

        self._logic.on_maneuver_goal_response(goal_handle.accepted, self._now_s())
        if not goal_handle.accepted:
            self._generacion_vigente = None
            return
        self._goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(
            lambda f: self._on_resultado(generacion, f))

    def _on_resultado(self, generacion, futuro):
        if generacion != self._generacion_vigente:
            return
        self._goal_handle = None
        self._generacion_vigente = None
        self._logic.on_maneuver_result(self._now_s())

    def _cancelar_maniobra(self):
        self._generacion_vigente = None
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

    def destroy_node(self):
        # El cliente de la acción se destruye de forma explícita: si se dejara para el
        # recolector de basura, podría liberarse después de cerrar rclpy y fallar.
        self._cliente.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
