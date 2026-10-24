"""Tests de la conversión de unidades SI al formato del ESP32 (ADR-0002)."""
import math

from bfmc_hardware_bridge.units import Esp32Limits, to_duty, to_servo
import pytest

LIMITES = Esp32Limits(max_speed_mps=0.5, max_steer_rad=math.radians(25.0), duty_start=140,
                      duty_max=255, servo_left=50, servo_center=105, servo_right=160)


# --- velocidad -------------------------------------------------------------------------

def test_velocidad_cero_es_duty_cero_sin_cambiar_el_sentido():
    assert to_duty(0.0, LIMITES) == (None, 0)


def test_la_velocidad_maxima_es_el_duty_maximo():
    assert to_duty(0.5, LIMITES) == (True, 255)


def test_marcha_atras_da_el_mismo_duty_con_el_sentido_invertido():
    assert to_duty(-0.25, LIMITES) == (False, to_duty(0.25, LIMITES)[1])


def test_la_mitad_de_la_velocidad_cae_a_mitad_de_camino_entre_arranque_y_maximo():
    assert to_duty(0.25, LIMITES) == (True, 198)  # 140 + 0.5 * 115 = 197.5


def test_cualquier_velocidad_distinta_de_cero_supera_el_umbral_de_arranque():
    assert to_duty(0.001, LIMITES)[1] >= 140
    assert to_duty(-0.001, LIMITES)[1] >= 140


def test_una_velocidad_excesiva_se_recorta_al_duty_maximo():
    assert to_duty(5.0, LIMITES) == (True, 255)
    assert to_duty(-5.0, LIMITES) == (False, 255)


@pytest.mark.parametrize('valor', [math.nan, math.inf, -math.inf])
def test_una_velocidad_no_finita_se_trata_como_cero(valor):
    assert to_duty(valor, LIMITES) == (None, 0)


# --- dirección -------------------------------------------------------------------------

def test_recto_es_el_centro_del_servo():
    assert to_servo(0.0, LIMITES) == 105


def test_positivo_es_izquierda_y_llega_al_tope_izquierdo():
    assert to_servo(math.radians(25.0), LIMITES) == 50


def test_negativo_es_derecha_y_llega_al_tope_derecho():
    assert to_servo(math.radians(-25.0), LIMITES) == 160


def test_cada_lado_escala_con_su_propio_recorrido():
    limites = Esp32Limits(0.5, math.radians(20.0), 140, 255, 60, 100, 120)
    assert to_servo(math.radians(10.0), limites) == 80   # mitad de 100 -> 60
    assert to_servo(math.radians(-10.0), limites) == 110  # mitad de 100 -> 120


def test_un_giro_excesivo_se_recorta_al_tope():
    assert to_servo(math.radians(90.0), LIMITES) == 50
    assert to_servo(math.radians(-90.0), LIMITES) == 160


@pytest.mark.parametrize('valor', [math.nan, math.inf, -math.inf])
def test_una_direccion_no_finita_se_trata_como_recto(valor):
    assert to_servo(valor, LIMITES) == 105


def test_el_servo_siempre_es_un_entero_dentro_del_recorrido():
    for grados in range(-40, 41):
        servo = to_servo(math.radians(grados), LIMITES)
        assert isinstance(servo, int) and 50 <= servo <= 160


# --- límites ---------------------------------------------------------------------------

@pytest.mark.parametrize('cambios', [
    {'max_speed_mps': 0.0},
    {'max_steer_rad': -1.0},
    {'duty_start': 0},
    {'duty_start': 255},
    {'duty_max': 256},
    {'servo_center': 50},
    {'servo_right': 100},
])
def test_limites_incoherentes_se_rechazan(cambios):
    base = {'max_speed_mps': 0.5, 'max_steer_rad': 0.4, 'duty_start': 140, 'duty_max': 255,
            'servo_left': 50, 'servo_center': 105, 'servo_right': 160}
    with pytest.raises(ValueError):
        Esp32Limits(**{**base, **cambios})
