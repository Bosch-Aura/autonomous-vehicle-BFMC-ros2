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

        # DTR y RTS se dejan en reposo ANTES de abrir: en el DevKit del ESP32 esas
        # líneas manejan EN y BOOT, y abrir el puerto con RTS activo deja la placa
        # retenida en reset. Aun así, con el driver de WCH de la Jetson abrir el
        # puerto reinicia la placa: la lógica lo detecta por EVENT:SYSTEM_READY.
        self._serial = serial.serial_for_url(ruta, baudrate=baud_rate, timeout=0,
                                             do_not_open=True)
        self._serial.dtr = False
        self._serial.rts = False
        self._serial.open()

    def write(self, datos: bytes) -> None:
        self._serial.write(datos)

    def read_available(self) -> bytes:
        pendientes = self._serial.in_waiting
        return self._serial.read(pendientes) if pendientes else b''

    def close(self) -> None:
        self._serial.close()
