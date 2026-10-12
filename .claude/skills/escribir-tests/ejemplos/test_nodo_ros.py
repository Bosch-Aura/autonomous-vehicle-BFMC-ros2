"""Ejemplo de test de integración de un nodo ROS2 con pytest.

Verifica que el nodo conecte mensajes con su lógica: publica un mensaje de
entrada y espera la salida. Como depende de la mensajería real, tiene un tope de
tiempo; la lógica de fondo se testea aparte, sin ROS (ver los otros ejemplos).

`DuplicaNode` está acá para que el ejemplo sea autocontenido. En el proyecto el
nodo vive en su paquete y el test lo importa.

Cómo correrlo (necesita ROS2 Humble, por ejemplo en el contenedor):

    source /opt/ros/humble/setup.bash
    pytest .claude/skills/escribir-tests/ejemplos/test_nodo_ros.py
"""
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64

TIMEOUT_S = 5.0


class DuplicaNode(Node):
    """Publica en `salida` el doble de lo recibido en `entrada`."""

    def __init__(self):
        super().__init__('duplica')
        self._publisher = self.create_publisher(Float64, 'salida', 10)
        self.create_subscription(Float64, 'entrada', self._on_input, 10)

    def _on_input(self, msg):
        out = Float64()
        out.data = msg.data * 2.0
        self._publisher.publish(out)


@pytest.fixture(scope='module')
def ros():
    """Un solo contexto ROS para todo el módulo: iniciarlo por test es lento."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def entorno(ros):
    """El nodo bajo prueba y un nodo auxiliar que actúa como cliente."""
    nodo = DuplicaNode()
    auxiliar = rclpy.create_node('auxiliar_de_test')
    executor = SingleThreadedExecutor()
    executor.add_node(nodo)
    executor.add_node(auxiliar)
    yield auxiliar, executor
    executor.shutdown()
    nodo.destroy_node()
    auxiliar.destroy_node()


def publicar_y_esperar_salida(auxiliar, executor, valor):
    """Publica `valor` hasta recibir respuesta o agotar el tope de tiempo.

    Reintenta porque el descubrimiento entre nodos no es instantáneo. Gira el
    executor en lugar de dormir: no hay `time.sleep` en el test.
    """
    recibidos = []
    auxiliar.create_subscription(
        Float64, 'salida', lambda msg: recibidos.append(msg.data), 10)
    publisher = auxiliar.create_publisher(Float64, 'entrada', 10)

    limite = time.monotonic() + TIMEOUT_S
    while not recibidos and time.monotonic() < limite:
        publisher.publish(Float64(data=valor))
        executor.spin_once(timeout_sec=0.1)
    return recibidos[0] if recibidos else None


def test_el_nodo_publica_el_doble_del_valor_recibido(entorno):
    auxiliar, executor = entorno
    assert publicar_y_esperar_salida(auxiliar, executor, 21.0) == 42.0


def test_el_nodo_conserva_el_signo_del_valor_recibido(entorno):
    auxiliar, executor = entorno
    assert publicar_y_esperar_salida(auxiliar, executor, -1.5) == -3.0
