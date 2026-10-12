"""Tests del protocolo serial Jetson <-> ESP32 (ADR-0001), sin ROS ni puerto."""
import json

import pytest

from bfmc_hardware_bridge.protocol import (
    encode_brake, encode_drive, parse_line, SEQ_MAX, StatusMessage, UltraMessage,
)


# --- lo que se envía al ESP32 -------------------------------------------------------

def test_un_comando_de_conduccion_tiene_el_formato_del_adr():
    assert encode_drive(12, 250, -30) == b'{"t":"drive","seq":12,"v":250,"s":-30}\n'


def test_un_comando_de_freno_tiene_el_formato_del_adr():
    assert encode_brake(13) == b'{"t":"brake","seq":13}\n'


def test_las_tramas_son_json_valido_de_una_sola_linea():
    for trama in (encode_drive(1, -500, 250), encode_brake(2)):
        assert trama.endswith(b'\n') and trama.count(b'\n') == 1
        assert isinstance(json.loads(trama.decode('ascii')), dict)


def test_los_campos_del_comando_de_conduccion_son_enteros():
    datos = json.loads(encode_drive(7, 100, -5))
    assert datos == {'t': 'drive', 'seq': 7, 'v': 100, 's': -5}
    assert all(isinstance(datos[clave], int) for clave in ('seq', 'v', 's'))


@pytest.mark.parametrize('seq', [-1, SEQ_MAX + 1])
def test_una_secuencia_fuera_de_rango_se_rechaza(seq):
    with pytest.raises(ValueError):
        encode_drive(seq, 0, 0)
    with pytest.raises(ValueError):
        encode_brake(seq)


def test_la_secuencia_maxima_cabe_en_los_nueve_digitos_que_lee_el_firmware():
    assert len(str(SEQ_MAX)) == 9
    assert encode_brake(SEQ_MAX) == b'{"t":"brake","seq":999999999}\n'


# --- lo que llega del ESP32 -------------------------------------------------------------

def test_una_lectura_valida_del_ultrasonico_se_interpreta():
    mensaje = parse_line('{"t":"ultra","id":"front","dist_cm":87,"valid":true}')
    assert mensaje == UltraMessage(dist_cm=87, valid=True)


def test_una_lectura_invalida_del_ultrasonico_se_interpreta():
    mensaje = parse_line('{"t":"ultra","id":"front","dist_cm":0,"valid":false}')
    assert mensaje == UltraMessage(dist_cm=0, valid=False)


def test_un_estado_del_esp32_se_interpreta():
    mensaje = parse_line('{"t":"status","fw":"0.1.0","wd":true,"last_seq":12}')
    assert mensaje == StatusMessage(firmware='0.1.0', watchdog_tripped=True, last_seq=12)


@pytest.mark.parametrize('linea', [
    '',
    'basura',
    '{',
    '[]',
    '42',
    'null',
    '{}',
    '{"t":"volar"}',
    '{"t":"ultra","id":"front","dist_cm":87}',                    # falta valid
    '{"t":"ultra","id":"trasero","dist_cm":87,"valid":true}',    # sensor desconocido
    '{"t":"ultra","id":"front","dist_cm":-1,"valid":true}',      # distancia negativa
    '{"t":"ultra","id":"front","dist_cm":"87","valid":true}',    # distancia como texto
    '{"t":"ultra","id":"front","dist_cm":true,"valid":true}',    # booleano no es entero
    '{"t":"ultra","id":"front","dist_cm":8.7,"valid":true}',     # decimal
    '{"t":"ultra","id":"front","dist_cm":87,"valid":"si"}',      # valid no es booleano
    '{"t":"status","fw":"0.1.0","wd":false}',                     # falta last_seq
    '{"t":"status","fw":1,"wd":false,"last_seq":1}',             # fw no es texto
    '{"t":"status","fw":"0.1.0","wd":"no","last_seq":1}',        # wd no es booleano
    '{"t":"status","fw":"0.1.0","wd":false,"last_seq":-3}',      # secuencia negativa
])
def test_una_linea_invalida_se_descarta_sin_lanzar_excepcion(linea):
    assert parse_line(linea) is None
