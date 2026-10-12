"""Ejemplo de test de contrato: lo que un módulo le manda a otro respeta el formato acordado.

El contrato acá es el del enlace Jetson -> ESP32: velocidad en mm/s y ángulo en
décimas de grado, ambos enteros y dentro de un rango. Un test de contrato falla
cuando alguien cambia el formato sin avisar al otro lado.

`a_comando_esp32` está acá para que el ejemplo sea autocontenido. En el
proyecto esta conversión vive en `bfmc_hardware_bridge` y el test la importa.
Los límites son los valores asumidos del ADR-0001 y se confirman en el auto.

Cómo correrlo (no necesita ROS):

    pytest .claude/skills/escribir-tests/ejemplos/test_contrato_comandos.py
"""
import math

import pytest

VELOCIDAD_MAX_MM_S = 500
DIRECCION_MAX_DECIMAS = 250


def a_comando_esp32(velocidad_m_s: float, direccion_rad: float):
    """Convierte unidades SI al formato del ESP32 y recorta al rango permitido."""
    mm_s = round(velocidad_m_s * 1000.0)
    decimas = round(math.degrees(direccion_rad) * 10.0)
    return (
        max(-VELOCIDAD_MAX_MM_S, min(VELOCIDAD_MAX_MM_S, mm_s)),
        max(-DIRECCION_MAX_DECIMAS, min(DIRECCION_MAX_DECIMAS, decimas)),
    )


@pytest.mark.parametrize(
    'velocidad_m_s, direccion_rad, esperado',
    [
        (0.0, 0.0, (0, 0)),
        (0.25, 0.0, (250, 0)),
        (-0.25, 0.0, (-250, 0)),
        (0.0, math.radians(10.0), (0, 100)),
        (0.0, math.radians(-10.0), (0, -100)),
    ],
    ids=['reposo', 'avance', 'marcha_atras', 'giro_izquierda', 'giro_derecha'],
)
def test_la_conversion_respeta_las_unidades_y_el_signo(
        velocidad_m_s, direccion_rad, esperado):
    assert a_comando_esp32(velocidad_m_s, direccion_rad) == esperado


def test_una_velocidad_excesiva_se_recorta_al_maximo():
    assert a_comando_esp32(5.0, 0.0)[0] == VELOCIDAD_MAX_MM_S
    assert a_comando_esp32(-5.0, 0.0)[0] == -VELOCIDAD_MAX_MM_S


def test_un_giro_excesivo_se_recorta_al_maximo():
    assert a_comando_esp32(0.0, math.radians(90.0))[1] == DIRECCION_MAX_DECIMAS
    assert a_comando_esp32(0.0, math.radians(-90.0))[1] == -DIRECCION_MAX_DECIMAS


def test_cualquier_entrada_produce_enteros_dentro_del_rango_del_contrato():
    """Barrido de entradas: el contrato vale para todas, no solo para los casos felices."""
    velocidades = [v / 4.0 for v in range(-40, 41)]
    direcciones = [d / 8.0 for d in range(-24, 25)]

    for velocidad in velocidades:
        for direccion in direcciones:
            mm_s, decimas = a_comando_esp32(velocidad, direccion)
            assert isinstance(mm_s, int) and isinstance(decimas, int)
            assert -VELOCIDAD_MAX_MM_S <= mm_s <= VELOCIDAD_MAX_MM_S
            assert -DIRECCION_MAX_DECIMAS <= decimas <= DIRECCION_MAX_DECIMAS
