"""Tests de la conversión de unidades SI al formato del ESP32 (ADR-0001)."""
import math

import pytest

from bfmc_hardware_bridge.units import to_esp32_units

MAX_V = 0.5
MAX_S = math.radians(25.0)


def convertir(velocidad, direccion):
    return to_esp32_units(velocidad, direccion, MAX_V, MAX_S)


@pytest.mark.parametrize('velocidad, direccion, esperado', [
    (0.0, 0.0, (0, 0)),
    (0.25, 0.0, (250, 0)),
    (-0.25, 0.0, (-250, 0)),
    (0.0, math.radians(10.0), (0, 100)),
    (0.0, math.radians(-10.0), (0, -100)),
    (0.1, math.radians(5.0), (100, 50)),
], ids=['reposo', 'avance', 'marcha_atras', 'izquierda', 'derecha', 'ambos'])
def test_la_conversion_respeta_unidades_y_signo(velocidad, direccion, esperado):
    assert convertir(velocidad, direccion) == esperado


def test_una_velocidad_excesiva_se_recorta_en_ambos_sentidos():
    assert convertir(5.0, 0.0)[0] == 500
    assert convertir(-5.0, 0.0)[0] == -500


def test_un_giro_excesivo_se_recorta_en_ambos_sentidos():
    assert convertir(0.0, math.radians(90.0))[1] == 250
    assert convertir(0.0, math.radians(-90.0))[1] == -250


def test_el_maximo_exacto_no_se_recorta():
    assert convertir(0.5, MAX_S) == (500, 250)


@pytest.mark.parametrize('valor', [math.nan, math.inf, -math.inf])
def test_una_velocidad_no_finita_se_trata_como_cero(valor):
    assert convertir(valor, 0.0) == (0, 0)


@pytest.mark.parametrize('valor', [math.nan, math.inf, -math.inf])
def test_una_direccion_no_finita_se_trata_como_cero(valor):
    assert convertir(0.25, valor) == (250, 0)


def test_la_salida_siempre_son_enteros_dentro_del_rango():
    for velocidad in [v / 4.0 for v in range(-40, 41)]:
        for direccion in [d / 8.0 for d in range(-24, 25)]:
            v, s = convertir(velocidad, direccion)
            assert isinstance(v, int) and isinstance(s, int)
            assert -500 <= v <= 500
            assert -250 <= s <= 250
