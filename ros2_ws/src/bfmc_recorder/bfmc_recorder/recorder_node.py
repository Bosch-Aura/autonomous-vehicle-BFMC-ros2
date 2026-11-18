"""Nodo bfmc_recorder: graba la cámara y el estado del sistema mientras se lo pide el visor.

Servicios (std_srvs/Trigger): `start_service` y `stop_service`. Publica en `status_topic` un
JSON con el estado de la grabación una vez por segundo. No maneja el auto ni lo frena: solo
escucha. Corta solo a los `max_minutes` o si queda poco espacio (`min_free_mb`).
"""
from datetime import datetime
import json
import os
import shutil
import subprocess
import threading

from ackermann_msgs.msg import AckermannDriveStamped
from bfmc_interfaces.msg import HardwareStatus, LaneState
from bfmc_recorder.recorder_logic import nombre_de_grabacion, Sesion, toca_guardar_cuadro
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

PARAMETROS_TEXTO = ('image_topic', 'lane_topic', 'lane_cmd_topic', 'drive_topic',
                    'hardware_status_topic', 'output_dir', 'start_service', 'stop_service',
                    'status_topic')
PARAMETROS_DECIMALES = ('max_fps', 'max_minutes', 'min_free_mb')
SEGUNDOS_DEL_VOLCADO = 15.0


def imagen_a_jpeg(msg: Image, calidad: int):
    """Codifica una Image (rgb8, bgr8 o mono8) como JPEG. None si el formato no se soporta."""
    canales = {'rgb8': 3, 'bgr8': 3, 'mono8': 1}.get(msg.encoding)
    if canales is None:
        return None
    datos = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    imagen = datos[:, :msg.width * canales].reshape(msg.height, msg.width, canales)
    if msg.encoding == 'rgb8':
        imagen = cv2.cvtColor(imagen, cv2.COLOR_RGB2BGR)
    ok, jpeg = cv2.imencode('.jpg', imagen, [cv2.IMWRITE_JPEG_QUALITY, int(calidad)])
    return jpeg.tobytes() if ok else None


class RecorderNode(Node):
    """Graba los cuadros y los mensajes del carril, del control y del hardware."""

    def __init__(self, **opciones):
        super().__init__('recorder', **opciones)
        for nombre in PARAMETROS_TEXTO:
            self.declare_parameter(nombre, Parameter.Type.STRING)
        for nombre in PARAMETROS_DECIMALES:
            self.declare_parameter(nombre, Parameter.Type.DOUBLE)
        self.declare_parameter('jpeg_quality', Parameter.Type.INTEGER)
        self.declare_parameter('meta_files', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('dump_nodes', Parameter.Type.STRING_ARRAY)

        self._sesion = None
        self._ultimo_cuadro_ns = None
        self._inicio_s = 0.0
        self._ultimo_motivo = ''

        mejor_esfuerzo = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, self._valor('image_topic'), self._on_imagen,
                                 mejor_esfuerzo)
        self.create_subscription(LaneState, self._valor('lane_topic'),
                                 self._evento('lane'), 10)
        self.create_subscription(AckermannDriveStamped, self._valor('lane_cmd_topic'),
                                 self._evento('lane_cmd'), 10)
        self.create_subscription(AckermannDriveStamped, self._valor('drive_topic'),
                                 self._evento('drive'), 10)
        self.create_subscription(HardwareStatus, self._valor('hardware_status_topic'),
                                 self._evento('hardware'), 10)
        self.create_service(Trigger, self._valor('start_service'), self._al_empezar)
        self.create_service(Trigger, self._valor('stop_service'), self._al_parar)
        self._estado = self.create_publisher(String, self._valor('status_topic'), 1)
        self.create_timer(1.0, self._publicar_estado)
        self.get_logger().info('recorder iniciado (no graba hasta que se lo pide)')

    def _valor(self, nombre):
        return self.get_parameter(nombre).value

    def _raiz(self):
        return os.path.expanduser(self._valor('output_dir'))

    def _libre_mb(self):
        raiz = self._raiz()
        os.makedirs(raiz, exist_ok=True)
        return shutil.disk_usage(raiz).free / 1e6

    # --- servicios ----------------------------------------------------------------

    def _al_empezar(self, _pedido, respuesta):
        if self._sesion is not None:
            respuesta.success, respuesta.message = False, 'ya está grabando'
            return respuesta
        try:
            if self._libre_mb() < self._valor('min_free_mb'):
                raise OSError('queda poco espacio libre en la Jetson')
            ahora = datetime.now()
            ruta = os.path.join(self._raiz(), nombre_de_grabacion(ahora))
            self._sesion = Sesion(ruta, ahora.isoformat(timespec='seconds'),
                                  self._valor('max_fps'), self._valor('jpeg_quality'))
        except OSError as error:
            respuesta.success, respuesta.message = False, 'no se pudo empezar: %s' % error
            return respuesta
        self._ultimo_cuadro_ns = None
        self._ultimo_motivo = ''
        self._inicio_s = self.get_clock().now().nanoseconds * 1e-9
        self._copiar_meta(ruta)
        self.get_logger().info('grabando en %s' % ruta)
        respuesta.success, respuesta.message = True, os.path.basename(ruta)
        return respuesta

    def _al_parar(self, _pedido, respuesta):
        if self._sesion is None:
            respuesta.success, respuesta.message = False, 'no estaba grabando'
            return respuesta
        nombre = self._cerrar('pedido del usuario')
        respuesta.success, respuesta.message = True, nombre
        return respuesta

    def _cerrar(self, motivo):
        sesion, self._sesion = self._sesion, None
        sesion.cerrar(datetime.now().isoformat(timespec='seconds'), motivo)
        self._ultimo_motivo = motivo
        self.get_logger().info('grabación cerrada (%s): %d cuadros en %s' % (
            motivo, sesion.cuadros, sesion.ruta))
        return os.path.basename(sesion.ruta)

    # --- datos --------------------------------------------------------------------

    def _on_imagen(self, msg: Image):
        if self._sesion is None:
            return
        ahora_ns = self.get_clock().now().nanoseconds
        if not toca_guardar_cuadro(self._ultimo_cuadro_ns, ahora_ns, self._valor('max_fps')):
            return
        jpeg = imagen_a_jpeg(msg, self._valor('jpeg_quality'))
        if jpeg is None:
            self.get_logger().error('encoding %r no soportado' % msg.encoding,
                                    throttle_duration_sec=10.0)
            return
        self._ultimo_cuadro_ns = ahora_ns
        instante = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self._sesion.guardar_cuadro(instante or ahora_ns, jpeg)

    def _evento(self, nombre):
        def guardar(msg):
            if self._sesion is None:
                return
            sello = getattr(getattr(msg, 'header', None), 'stamp', None)
            stamp_ns = None if sello is None else sello.sec * 1_000_000_000 + sello.nanosec
            self._sesion.guardar_evento(
                nombre, self.get_clock().now().nanoseconds, stamp_ns,
                dict(message_to_ordereddict(msg)))
        return guardar

    def _copiar_meta(self, ruta):
        """Copia los archivos de parámetros y vuelca los de los nodos (sin frenar el nodo)."""
        for origen in self._valor('meta_files'):
            origen = os.path.expanduser(origen)
            if os.path.exists(origen):
                shutil.copy(origen, os.path.join(ruta, 'meta_' + os.path.basename(origen)))
        for nodo in self._valor('dump_nodes'):
            threading.Thread(target=self._volcar, args=(nodo, ruta), daemon=True).start()

    @staticmethod
    def _volcar(nodo, ruta):
        destino = os.path.join(ruta, 'params_%s.yaml' % nodo)
        try:
            with open(destino, 'w', encoding='utf-8') as archivo:
                subprocess.run(['ros2', 'param', 'dump', '/' + nodo], stdout=archivo,
                               stderr=subprocess.DEVNULL, timeout=SEGUNDOS_DEL_VOLCADO,
                               check=False)
        except (OSError, subprocess.SubprocessError):
            pass

    def _publicar_estado(self):
        grabando = self._sesion is not None
        segundos = 0.0
        if grabando:
            segundos = self.get_clock().now().nanoseconds * 1e-9 - self._inicio_s
            libre = self._libre_mb()
            if segundos >= self._valor('max_minutes') * 60.0:
                self._cerrar('tiempo máximo')
                grabando = False
            elif libre < self._valor('min_free_mb'):
                self._cerrar('poco espacio libre')
                grabando = False
        datos = {'grabando': grabando, 'segundos': round(segundos, 1),
                 'motivo_fin': self._ultimo_motivo, 'carpeta': self._raiz()}
        sesion = self._sesion
        if sesion is not None:
            datos.update(nombre=os.path.basename(sesion.ruta), cuadros=sesion.cuadros,
                         mb=round(sesion.bytes / 1e6, 1))
        self._estado.publish(String(data=json.dumps(datos)))


def main(args=None):
    rclpy.init(args=args)
    nodo = RecorderNode()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        if nodo._sesion is not None:
            nodo._cerrar('el nodo se cerró')
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
