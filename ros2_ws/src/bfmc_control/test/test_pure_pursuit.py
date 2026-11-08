"""Tests de Pure Pursuit (pure_pursuit.py) y de su uso en ControlLogic, sin ROS.

Estados de carril sintéticos. Convención: positivo a la izquierda (bfmc_interfaces).
"""
import math

from bfmc_control.control_logic import ControlConfig, ControlLogic, LaneInput
from bfmc_control.pure_pursuit import (
    angulo_de_direccion, distancia_de_mirada, punto_objetivo, PurePursuitConfig, velocidad,
)
import pytest

PP = PurePursuitConfig(
    wheelbase_m=0.26, camera_to_rear_axle_m=0.20, lookahead_min_m=0.70,
    lookahead_max_m=1.10, lookahead_gain_s=0.0, max_lateral_accel_mps2=1.0,
    speed_min_factor=0.5)
# El detector mide a 0,575 m del pie de la cámara: 0,775 m del eje trasero.
REFERENCIA_M = 0.575
MAX_STEER = 0.5236


def giro(lateral=0.0, rumbo=0.0, curvatura=0.0, pp=PP):
    ld = distancia_de_mirada(pp, 0.3)
    x, y = punto_objetivo(pp, lateral, rumbo, curvatura, REFERENCIA_M, ld)
    return angulo_de_direccion(pp, x, y)


def logica(max_steer_rad=MAX_STEER, **pp):
    base = {
        'kp': 1.0, 'ki': 0.0, 'kd': 0.0, 'heading_weight_m_per_rad': 0.0,
        'deadband_m': 0.0, 'integral_limit_rad': 1.0, 'max_steer_rad': max_steer_rad,
        'max_steer_rate_rad_s': 1000.0, 'filter_alpha': 1.0, 'min_confidence': 0.3,
        'cruise_speed_mps': 0.3, 'lost_speed_factor': 1.0, 'slowdown_gain': 0.0,
        'max_dt_s': 1.0,
    }
    config = PurePursuitConfig(**{**PP.__dict__, **pp})
    return ControlLogic(ControlConfig(**base, algorithm='pure_pursuit', pure_pursuit=config))


def carril(lateral=0.0, rumbo=0.0, curvatura=0.0, confianza=1.0):
    return LaneInput(True, lateral, rumbo, confianza, curvature=curvatura,
                     reference_distance_m=REFERENCIA_M)


# --- criterios del ticket ------------------------------------------------------------------

def test_carril_recto_y_auto_centrado_no_gira():
    assert giro() == pytest.approx(0.0)


def test_auto_corrido_a_la_derecha_gira_a_la_izquierda():
    # El centro del carril queda a la izquierda del auto: lateral_error_m positivo.
    assert giro(lateral=0.05) > 0.0


def test_auto_corrido_a_la_izquierda_gira_a_la_derecha():
    assert giro(lateral=-0.05) < 0.0


def test_curva_a_la_izquierda_con_el_auto_centrado_gira_a_la_izquierda():
    # Con Ld más allá del punto de medición, el carril que dobla se ve a la izquierda.
    lejos = PurePursuitConfig(**{**PP.__dict__, 'lookahead_min_m': 1.0})
    assert giro(rumbo=0.1, curvatura=1.0, pp=lejos) > 0.0


def test_el_giro_se_satura_en_el_tope_del_servo():
    # Sin tope, con 0,5 m de desvío Pure Pursuit pide ~0,34 rad.
    assert giro(lateral=0.5) > 0.3
    salida = logica(max_steer_rad=0.1).update(carril(lateral=0.5), now=0.0)
    assert salida.steering_rad == pytest.approx(0.1)


@pytest.mark.parametrize('velocidad_mps, esperada', [(0.0, 0.70), (0.5, 0.80), (5.0, 1.10)])
def test_ld_respeta_los_limites(velocidad_mps, esperada):
    proporcional = PurePursuitConfig(**{**PP.__dict__, 'lookahead_gain_s': 1.6})
    assert distancia_de_mirada(proporcional, velocidad_mps) == pytest.approx(esperada)


def test_con_ganancia_cero_ld_queda_fija_en_la_minima():
    assert distancia_de_mirada(PP, 3.0) == pytest.approx(PP.lookahead_min_m)


# --- geometría -----------------------------------------------------------------------------

def test_el_punto_objetivo_se_traslada_desde_donde_se_midio():
    # Medido a 0,775 m del eje trasero; el objetivo a 1,0 m está 0,225 m más adelante.
    x, y = punto_objetivo(PP, 0.05, 0.1, 0.0, REFERENCIA_M, 1.0)
    assert x == 1.0
    assert y == pytest.approx(0.05 + 0.1 * 0.225)


def test_la_ley_de_control_es_la_de_pure_pursuit():
    x, y = 0.8, 0.1
    esperado = math.atan(2 * 0.26 * math.sin(math.atan2(y, x)) / math.hypot(x, y))
    assert angulo_de_direccion(PP, x, y) == pytest.approx(esperado)


# --- velocidad -----------------------------------------------------------------------------

def test_en_recta_y_con_confianza_va_a_velocidad_de_crucero():
    assert velocidad(PP, 0.3, 0.0, 1.0) == pytest.approx(0.3)


def test_en_una_curva_cerrada_baja_para_no_pasar_la_aceleracion_lateral():
    # Radio 0,1 m: v = sqrt(1,0 / 10) = 0,316 m/s; con crucero 0,5 m/s, limita.
    assert velocidad(PP, 0.5, 10.0, 1.0) == pytest.approx(math.sqrt(0.1))


def test_con_poca_confianza_baja_hasta_el_piso():
    assert velocidad(PP, 0.3, 0.0, 0.7) == pytest.approx(0.21)
    assert velocidad(PP, 0.3, 0.0, 0.1) == pytest.approx(0.15)     # piso: 50 %


# --- integración en ControlLogic -----------------------------------------------------------

def test_con_pure_pursuit_el_control_usa_su_giro_y_su_velocidad():
    salida = logica().update(carril(lateral=0.05, confianza=0.7), now=0.0)
    assert salida.steering_rad == pytest.approx(giro(lateral=0.05))
    assert salida.speed_mps == pytest.approx(0.21)


def test_sin_carril_hace_lo_mismo_que_con_el_pid():
    salida = logica().update(LaneInput(False, 0.0, 0.0, 0.0), now=0.0)
    assert salida.steering_rad == 0.0
    assert salida.speed_mps == pytest.approx(0.3)      # lost_speed_factor 1.0


def test_un_algoritmo_desconocido_se_rechaza():
    with pytest.raises(ValueError):
        ControlConfig(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.5, 1.0, 1.0, 0.3, 0.3, 1.0, 0.0, 1.0,
                      algorithm='stanley')


def test_pure_pursuit_sin_parametros_se_rechaza():
    with pytest.raises(ValueError):
        ControlConfig(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.5, 1.0, 1.0, 0.3, 0.3, 1.0, 0.0, 1.0,
                      algorithm='pure_pursuit')


@pytest.mark.parametrize('cambio', [
    {'wheelbase_m': 0.0}, {'lookahead_min_m': 1.2}, {'max_lateral_accel_mps2': 0.0},
    {'speed_min_factor': 0.0}, {'lookahead_gain_s': -1.0},
])
def test_parametros_invalidos_se_rechazan(cambio):
    with pytest.raises(ValueError):
        PurePursuitConfig(**{**PP.__dict__, **cambio})
