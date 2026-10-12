"""Tests del armado de líneas a partir de los bytes del puerto serial."""
from bfmc_hardware_bridge.line_buffer import LineBuffer
import pytest


def test_una_linea_completa_se_entrega_sin_el_terminador():
    assert LineBuffer().feed(b'hola\n') == ['hola']


def test_una_linea_partida_en_dos_lecturas_se_completa():
    buffer = LineBuffer()
    assert buffer.feed(b'{"t":"ul') == []
    assert buffer.feed(b'tra"}\n') == ['{"t":"ultra"}']


def test_varias_lineas_en_una_sola_lectura_se_entregan_todas():
    assert LineBuffer().feed(b'uno\ndos\ntres\n') == ['uno', 'dos', 'tres']


def test_el_retorno_de_carro_se_ignora():
    assert LineBuffer().feed(b'hola\r\n') == ['hola']


def test_las_lineas_vacias_no_se_entregan():
    assert LineBuffer().feed(b'\n\r\n  \nhola\n') == ['hola']


def test_una_linea_incompleta_queda_esperando_sin_entregarse():
    buffer = LineBuffer()
    buffer.feed(b'parte')
    assert buffer.feed(b'') == []


def test_una_linea_demasiado_larga_se_descarta_completa():
    buffer = LineBuffer(max_line=8)
    assert buffer.feed(b'x' * 50 + b'\n') == []
    assert buffer.feed(b'ok\n') == ['ok']


def test_los_bytes_que_no_son_texto_no_rompen_el_armado():
    lineas = LineBuffer().feed(b'\xff\xfe{"t":1}\nsiguiente\n')
    assert len(lineas) == 2
    assert lineas[1] == 'siguiente'


def test_un_largo_maximo_no_positivo_se_rechaza():
    with pytest.raises(ValueError):
        LineBuffer(max_line=0)
