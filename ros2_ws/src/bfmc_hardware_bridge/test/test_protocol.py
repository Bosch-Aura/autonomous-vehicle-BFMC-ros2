"""Tests del protocolo serial Jetson <-> ESP32 (ADR-0002), sin ROS ni puerto."""
from bfmc_hardware_bridge.protocol import (
    encode_arm, encode_brake_now, encode_direction, encode_disarm, encode_resume,
    encode_speed, encode_steer, encode_stop, Event, parse_line,
)
import pytest


# --- lo que se envía al ESP32 -------------------------------------------------------

@pytest.mark.parametrize('trama, esperada', [
    (encode_speed(150), b'C:SET_SPEED:150\n'),
    (encode_speed(0), b'C:SET_SPEED:0\n'),
    (encode_steer(105), b'C:SET_STEER:105\n'),
    (encode_direction(True), b'C:SET_DIR:1\n'),
    (encode_direction(False), b'C:SET_DIR:0\n'),
    (encode_brake_now(), b'E:BRAKE_NOW:0\n'),
    (encode_arm(), b'M:SYS_ARM:0\n'),
    (encode_disarm(), b'M:SYS_DISARM:0\n'),
    (encode_stop(), b'M:SYS_STOP:0\n'),
    (encode_resume(), b'M:SYS_RESUME:0\n'),
])
def test_cada_comando_tiene_el_formato_canal_comando_valor_del_firmware(trama, esperada):
    assert trama == esperada


def test_cada_trama_es_una_sola_linea_ascii():
    for trama in (encode_speed(255), encode_steer(50), encode_arm(), encode_brake_now()):
        assert trama.endswith(b'\n') and trama.count(b'\n') == 1
        trama.decode('ascii')


# --- lo que se recibe del ESP32 -----------------------------------------------------

def test_un_evento_sin_argumentos_se_interpreta():
    assert parse_line('EVENT:SYSTEM_READY') == Event('SYSTEM_READY', ())


def test_un_evento_con_argumentos_los_separa():
    assert parse_line('EVENT:CMD_RECEIVED:SET_SPEED:150') == Event(
        'CMD_RECEIVED', ('SET_SPEED', '150'))


def test_los_argumentos_con_flecha_se_conservan():
    assert parse_line('EVENT:STATE_AUTO_TRANSITION:ARMED->RUNNING') == Event(
        'STATE_AUTO_TRANSITION', ('ARMED->RUNNING',))


@pytest.mark.parametrize('linea', [
    '[MotorTask] Motor task started',
    '========================================',
    'ESP32 RC Car FreeRTOS System Starting...',
    'EVENT:',
    'EVENT:minusculas',
    'event:SYSTEM_READY',
    '',
    '��EVENT:SYSTEM_READY',
])
def test_lo_que_no_es_un_evento_se_descarta(linea):
    assert parse_line(linea) is None
