"""Tests del puerto real con la URL loop:// de pyserial, que no necesita hardware.

loop:// devuelve por lectura lo que se escribió: sirve para comprobar que la
clase usa bien pyserial. Que el ESP32 real responda solo se prueba con el auto.
"""
import pytest

from bfmc_hardware_bridge.serial_port import PySerialPort


@pytest.fixture
def puerto():
    puerto = PySerialPort('loop://', 115200)
    yield puerto
    puerto.close()


def test_lo_que_se_escribe_se_puede_leer(puerto):
    puerto.write(b'{"t":"brake","seq":1}\n')
    assert puerto.read_available() == b'{"t":"brake","seq":1}\n'


def test_sin_datos_pendientes_la_lectura_no_espera_y_devuelve_vacio(puerto):
    assert puerto.read_available() == b''


def test_una_lectura_entrega_todo_lo_acumulado(puerto):
    puerto.write(b'uno\n')
    puerto.write(b'dos\n')
    assert puerto.read_available() == b'uno\ndos\n'


def test_un_puerto_que_no_existe_lanza_oserror():
    with pytest.raises(OSError):
        PySerialPort('/dev/este_puerto_no_existe', 115200)
