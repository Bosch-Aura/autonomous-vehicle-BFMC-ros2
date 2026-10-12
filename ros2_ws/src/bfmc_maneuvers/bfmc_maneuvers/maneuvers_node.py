"""Nodo ROS2 de bfmc_maneuvers: action server que ejecuta maniobras puntuales."""
import threading
import time

from ackermann_msgs.msg import AckermannDriveStamped
from bfmc_interfaces.action import ManeuverExecute
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy

from bfmc_maneuvers.strategies.base import DriveOutput
from bfmc_maneuvers.strategies.factory import build_strategy, ManeuverParams
from bfmc_maneuvers.strategies.intersection import IntersectionParams, TurnPlan
from bfmc_maneuvers.strategies.stop import StopParams

PARAMETROS_TEXTO = ('action_name', 'cmd_topic', 'frame_id')
PARAMETROS_DECIMALES = (
    'rate_hz',
    'stop.hold_s', 'stop.exit_s', 'stop.exit_speed_mps',
    'intersection.speed_mps', 'intersection.straight_s',
    'intersection.left.approach_s', 'intersection.left.turn_s', 'intersection.left.steer_rad',
    'intersection.right.approach_s', 'intersection.right.turn_s',
    'intersection.right.steer_rad',
)


class ManeuversNode(Node):
    """Ejecuta una maniobra cuando se la piden por la acción `ManeuverExecute`.

    Mientras la maniobra está activa publica su comando en `cmd_topic`. Solo una
    maniobra puede estar activa a la vez: un pedido nuevo se rechaza. Se puede
    cancelar en cualquier momento.

    Todos los parámetros se declaran sin valor por defecto: los valores viven en
    config/params.yaml y, si falta alguno, el nodo falla al arrancar.
    """

    def __init__(self):
        super().__init__('maneuvers')

        for nombre in PARAMETROS_TEXTO:
            self.declare_parameter(nombre, Parameter.Type.STRING)
        for nombre in PARAMETROS_DECIMALES:
            self.declare_parameter(nombre, Parameter.Type.DOUBLE)

        self._frame_id = self._valor('frame_id')
        self._period_s = 1.0 / self._valor('rate_hz')
        self._params = self._leer_parametros()

        self._lock = threading.Lock()
        self._active = False

        self._publisher = self.create_publisher(
            AckermannDriveStamped, self._valor('cmd_topic'),
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self._server = ActionServer(
            self, ManeuverExecute, self._valor('action_name'),
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=ReentrantCallbackGroup())

        self.get_logger().info('maneuvers iniciado')

    def _valor(self, nombre):
        return self.get_parameter(nombre).value

    def _leer_parametros(self):
        def giro(prefijo):
            return TurnPlan(
                approach_s=self._valor(prefijo + '.approach_s'),
                turn_s=self._valor(prefijo + '.turn_s'),
                steer_rad=self._valor(prefijo + '.steer_rad'))

        return ManeuverParams(
            stop=StopParams(
                hold_s=self._valor('stop.hold_s'),
                exit_s=self._valor('stop.exit_s'),
                exit_speed_mps=self._valor('stop.exit_speed_mps')),
            intersection=IntersectionParams(
                speed_mps=self._valor('intersection.speed_mps'),
                straight_s=self._valor('intersection.straight_s'),
                left=giro('intersection.left'),
                right=giro('intersection.right')))

    def _now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    # --- acción -----------------------------------------------------------------
    def _on_goal(self, goal_request):
        try:
            build_strategy(goal_request.maneuver, goal_request.direction, self._params)
        except ValueError as error:
            self.get_logger().warning('pedido rechazado: %s' % error)
            return GoalResponse.REJECT
        with self._lock:
            if self._active:
                self.get_logger().warning('pedido rechazado: ya hay una maniobra activa')
                return GoalResponse.REJECT
            self._active = True
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        pedido = goal_handle.request
        resultado = ManeuverExecute.Result()
        try:
            strategy = build_strategy(pedido.maneuver, pedido.direction, self._params)
            inicio = self._now_s()
            strategy.start(inicio)
            self.get_logger().info(
                'maniobra iniciada: %s %s' % (pedido.maneuver, pedido.direction))

            while True:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    resultado.success = False
                    resultado.message = 'cancelada'
                    return resultado

                ahora = self._now_s()
                paso = strategy.tick(ahora)
                if paso.finished:
                    break
                self._publicar(paso.output)

                realimentacion = ManeuverExecute.Feedback()
                realimentacion.elapsed_s = ahora - inicio
                goal_handle.publish_feedback(realimentacion)
                time.sleep(self._period_s)

            goal_handle.succeed()
            resultado.success = True
            resultado.message = 'completada'
            return resultado
        finally:
            # Al terminar o cancelar se publica un comando de parada, para que nadie
            # quede con el último comando de la maniobra.
            self._publicar(DriveOutput(speed=0.0, steering=0.0))
            with self._lock:
                self._active = False

    def destroy_node(self):
        # El servidor de la acción se destruye de forma explícita: si se dejara para el
        # recolector de basura, podría liberarse después de cerrar rclpy y fallar.
        self._server.destroy()
        super().destroy_node()

    def _publicar(self, salida: DriveOutput):
        comando = AckermannDriveStamped()
        comando.header.stamp = self.get_clock().now().to_msg()
        comando.header.frame_id = self._frame_id
        comando.drive.speed = salida.speed
        comando.drive.steering_angle = salida.steering
        self._publisher.publish(comando)


def main(args=None):
    rclpy.init(args=args)
    node = ManeuversNode()
    # La acción necesita varios hilos: el pedido, la cancelación y la ejecución
    # se atienden a la vez.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
