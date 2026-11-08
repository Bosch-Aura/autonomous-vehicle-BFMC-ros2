"""Mide la latencia de punta a punta: de la imagen de la cámara a /cmd/drive (A2-155).

El header.stamp de /cmd/drive es el de la imagen con la que se decidió (lo propagan
bfmc_control y bfmc_decision), así la latencia es "ahora - stamp" al recibirlo.

    ros2 run bfmc_bringup medir_latencia              # 30 s, en /cmd/drive
    ros2 run bfmc_bringup medir_latencia --ros-args -p duracion_s:=60.0

Imprime cantidad, promedio, p99 y máximo. Anotar también el modo de energía de la Jetson
(`sudo nvpmodel -q`). Requisito: que el reloj de la cámara sea el del sistema
(`global_time_enabled` en realsense-ros); si no, la resta no tiene sentido.

Los comandos con el auto detenido llevan el instante en que se publican (no hay imagen de
origen) y darían latencia ~0: se miden solo los que tienen velocidad.
"""
import math
from typing import Sequence

from ackermann_msgs.msg import AckermannDriveStamped
import rclpy
from rclpy.node import Node


def resumen(latencias_s: Sequence[float]) -> dict:
    """Cantidad, promedio, p99 y máximo [s] de las latencias. Lógica pura."""
    if not latencias_s:
        raise ValueError('no hay mediciones')
    ordenadas = sorted(latencias_s)
    # p99 por el método del rango más cercano: el valor que deja el 99 % por debajo o igual.
    indice = max(0, math.ceil(0.99 * len(ordenadas)) - 1)
    return {'cantidad': len(ordenadas), 'promedio_s': sum(ordenadas) / len(ordenadas),
            'p99_s': ordenadas[indice], 'maximo_s': ordenadas[-1]}


class MedidorDeLatencia(Node):
    """Junta "ahora - header.stamp" de cada comando en movimiento durante `duracion_s`."""

    def __init__(self):
        super().__init__('medir_latencia')
        self.declare_parameter('topic', '/cmd/drive')
        self.declare_parameter('duracion_s', 30.0)
        self.latencias = []
        self.create_subscription(AckermannDriveStamped, self.get_parameter('topic').value,
                                 self._on_comando, 10)
        self._fin = (self.get_clock().now().nanoseconds / 1e9
                     + self.get_parameter('duracion_s').value)

    def terminado(self) -> bool:
        return self.get_clock().now().nanoseconds / 1e9 >= self._fin

    def _on_comando(self, msg: AckermannDriveStamped):
        if msg.drive.speed == 0.0:
            return
        ahora = self.get_clock().now().nanoseconds / 1e9
        self.latencias.append(ahora - (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9))


def main(args=None):
    rclpy.init(args=args)
    nodo = MedidorDeLatencia()
    try:
        while rclpy.ok() and not nodo.terminado():
            rclpy.spin_once(nodo, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    try:
        datos = resumen(nodo.latencias)
        print('comandos: %d  promedio: %.1f ms  p99: %.1f ms  máximo: %.1f ms' % (
            datos['cantidad'], 1000 * datos['promedio_s'], 1000 * datos['p99_s'],
            1000 * datos['maximo_s']))
    except ValueError:
        print('no llegó ningún comando con el auto en movimiento')
    nodo.destroy_node()
    rclpy.try_shutdown()

