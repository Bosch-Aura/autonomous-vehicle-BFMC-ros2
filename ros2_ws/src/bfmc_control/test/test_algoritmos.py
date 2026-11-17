"""Tests de los algoritmos de giro intercambiables (ADR-0004), sin ROS.

El contrato que cumple todo algoritmo del REGISTRO se prueba una vez para todos: si alguien
agrega uno que no respeta los signos de bfmc_interfaces, estos tests fallan.
"""
import math

from bfmc_control import algoritmos
from bfmc_control.algoritmos.pure_pursuit import mirada_segun_curvatura
from bfmc_control.control_logic import ALGORITMOS, ControlConfig, ControlLogic, LaneInput
from bfmc_control.pure_pursuit import PurePursuitConfig
import pytest

PP = PurePursuitConfig(
    wheelbase_m=0.26, camera_to_rear_axle_m=0.20, lookahead_min_m=0.80,
    lookahead_max_m=1.10, lookahead_gain_s=0.0, max_lateral_accel_mps2=1.0,
    speed_min_factor=0.5, lookahead_curve_m=0.55, lookahead_curve_kappa=2.0)
REFERENCIA_M = 0.575


def configuracion(algoritmo, **cambios):
    base = {
        'kp': 1.5, 'ki': 0.0, 'kd': 0.0, 'heading_weight_m_per_rad': 0.5,
        'deadband_m': 0.0, 'integral_limit_rad': 1.0, 'max_steer_rad': math.radians(25.0),
        'max_steer_rate_rad_s': 1000.0, 'filter_alpha': 1.0, 'min_confidence': 0.15,
        'cruise_speed_mps': 0.3, 'lost_speed_factor': 1.0, 'slowdown_gain': 0.0,
        'max_dt_s': 1.0, 'curvature_ff_gain': 0.7, 'curvature_ff_min_confidence': 0.9,
        'curvature_ff_max': 2.2, 'curvature_ff_hold_s': 0.5, 'algorithm': algoritmo,
        'pure_pursuit': PP,
    }
    return ControlConfig(**{**base, **cambios})


def carril(lateral=0.0, rumbo=0.0, curvatura=0.0, confianza=1.0):
    return LaneInput(True, lateral, rumbo, confianza, curvature=curvatura,
                     reference_distance_m=REFERENCIA_M)


def giro(algoritmo, **kwargs):
    return ControlLogic(configuracion(algoritmo)).update(carril(**kwargs), now=0.0).steering_rad


# --- registro ------------------------------------------------------------------------------

def test_el_registro_tiene_los_algoritmos_que_acepta_la_configuracion():
    assert set(algoritmos.REGISTRO) == set(ALGORITMOS)


def test_un_nombre_desconocido_se_rechaza():
    with pytest.raises(ValueError):
        algoritmos.crear('stanley')


# --- contrato: todos los algoritmos --------------------------------------------------------

@pytest.mark.parametrize('algoritmo', ALGORITMOS)
def test_centrado_en_una_recta_no_gira(algoritmo):
    assert giro(algoritmo) == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize('algoritmo', ALGORITMOS)
def test_el_carril_a_la_izquierda_gira_a_la_izquierda(algoritmo):
    assert giro(algoritmo, lateral=0.05) > 0.0
    assert giro(algoritmo, lateral=-0.05) < 0.0


@pytest.mark.parametrize('algoritmo', ALGORITMOS)
def test_una_curva_a_la_izquierda_gira_a_la_izquierda(algoritmo):
    assert giro(algoritmo, curvatura=1.5) > 0.0
    assert giro(algoritmo, curvatura=-1.5) < 0.0


@pytest.mark.parametrize('algoritmo', ALGORITMOS)
def test_el_giro_nunca_pasa_el_tope(algoritmo):
    assert abs(giro(algoritmo, lateral=1.0, curvatura=5.0)) <= math.radians(25.0) + 1e-9


@pytest.mark.parametrize('algoritmo', ALGORITMOS)
def test_la_velocidad_es_positiva_y_no_pasa_la_de_crucero(algoritmo):
    salida = ControlLogic(configuracion(algoritmo)).update(carril(curvatura=1.0), now=0.0)
    assert 0.0 < salida.speed_mps <= 0.3


# --- Pure Pursuit: Ld según la curvatura ---------------------------------------------------

def test_en_recta_ld_es_la_de_recta():
    assert mirada_segun_curvatura(PP, 0.80, 0.0) == pytest.approx(0.80)


def test_en_la_curva_de_referencia_ld_es_la_de_curva():
    assert mirada_segun_curvatura(PP, 0.80, 2.0) == pytest.approx(0.55)
    assert mirada_segun_curvatura(PP, 0.80, -3.0) == pytest.approx(0.55)


def test_a_mitad_de_camino_ld_queda_en_el_medio():
    assert mirada_segun_curvatura(PP, 0.80, 1.0) == pytest.approx(0.675)


def test_con_lookahead_curve_m_en_cero_ld_no_cambia():
    pp = PurePursuitConfig(**{**PP.__dict__, 'lookahead_curve_m': 0.0})
    assert mirada_segun_curvatura(pp, 0.80, 2.0) == pytest.approx(0.80)


def test_en_una_curva_ld_corta_gira_mas_que_ld_fija():
    fija = PurePursuitConfig(**{**PP.__dict__, 'lookahead_curve_m': 0.0})
    corta = ControlLogic(configuracion('pure_pursuit')).update(carril(curvatura=2.0), 0.0)
    larga = ControlLogic(configuracion('pure_pursuit', pure_pursuit=fija)).update(
        carril(curvatura=2.0), 0.0)
    assert corta.steering_rad > larga.steering_rad > 0.0


def test_pure_pursuit_ignora_la_curvatura_de_una_sola_linea():
    # Una línea da confianza 0,5 (< curvature_ff_min_confidence): su curvatura es ruido.
    con_ruido = giro('pure_pursuit', curvatura=2.0, confianza=0.5)
    sin_curva = giro('pure_pursuit', confianza=0.5)
    assert con_ruido == pytest.approx(sin_curva)


# --- comunes: límite físico y cambio de algoritmo -----------------------------------------

def test_la_curvatura_maxima_sale_del_tope_y_la_distancia_entre_ejes():
    logic = ControlLogic(configuracion('pid'))
    assert logic.curvatura_maxima() == pytest.approx(math.tan(math.radians(25.0)) / 0.26)


@pytest.mark.parametrize('algoritmo', ALGORITMOS)
def test_una_curva_imposible_baja_la_velocidad(algoritmo):
    cfg = configuracion(algoritmo, infeasible_curve_speed_factor=0.5, curvature_ff_max=3.0)
    posible = ControlLogic(cfg).update(carril(curvatura=1.0), 0.0).speed_mps
    imposible = ControlLogic(cfg).update(carril(curvatura=2.5), 0.0).speed_mps
    assert imposible < posible
    assert imposible == pytest.approx(0.5 * ControlLogic(configuracion(
        algoritmo, curvature_ff_max=3.0)).update(carril(curvatura=2.5), 0.0).speed_mps)


def test_una_curva_imposible_con_una_sola_linea_no_frena():
    cfg = configuracion('pid', infeasible_curve_speed_factor=0.5)
    salida = ControlLogic(cfg).update(carril(curvatura=5.0, confianza=0.5), 0.0)
    assert salida.speed_mps == pytest.approx(0.3)


def test_cambiar_de_algoritmo_arranca_el_nuevo_limpio():
    logic = ControlLogic(configuracion('pid', ki=1.0))
    for paso in range(10):
        logic.update(carril(lateral=0.1), now=paso * 0.05)
    anterior = logic.algoritmo
    logic.set_config(configuracion('pure_pursuit', ki=1.0))
    assert logic.algoritmo is not anterior
    assert type(logic.algoritmo).__name__ == 'PurePursuit'


def test_cambiar_una_ganancia_no_cambia_de_instancia():
    logic = ControlLogic(configuracion('pid'))
    anterior = logic.algoritmo
    logic.set_config(configuracion('pid', kp=2.0))
    assert logic.algoritmo is anterior
