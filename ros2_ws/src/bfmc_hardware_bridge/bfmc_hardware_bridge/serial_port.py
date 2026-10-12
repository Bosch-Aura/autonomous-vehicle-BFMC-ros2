"""Puerto serial inyectable.

La lógica del puente no conoce el puerto real: usa cualquier objeto con estos
métodos. Así se testea con un puerto simulado, sin hardware.
"""
from typing import Protocol


class SerialPort(Protocol):
    """Lo que el nodo necesita de un puerto serial."""

    def write(self, datos: bytes) -> None:
        """Envía bytes. Lanza OSError si el puerto falló."""

    def read_available(self) -> bytes:
        """Devuelve los bytes disponibles sin esperar. Lanza OSError si el puerto falló."""

    def close(self) -> None:
        """Cierra el puerto."""


class PySerialPort:
    """Puerto real, con pyserial.

    `ruta` es un dispositivo (por ejemplo /dev/ttyUSB0) o una URL de pyserial
    (por ejemplo loop://, que sirve para probar sin hardware).
    """

    def __init__(self, ruta: str, baud_rate: int):
        import serial  # se importa acá para que el resto del paquete no lo requiera

        self._serial = serial.serial_for_url(ruta, baudrate=baud_rate, timeout=0)

    def write(self, datos: bytes) -> None:
        self._serial.write(datos)

    def read_available(self) -> bytes:
        pendientes = self._serial.in_waiting
        return self._serial.read(pendientes) if pendientes else b''

    def close(self) -> None:
        self._serial.close()
