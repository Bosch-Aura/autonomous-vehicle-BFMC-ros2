"""Tests de la lógica del control: sin ROS y con el tiempo simulado."""
from dataclasses import replace

import math

from bfmc_control.control_logic import (
    ControlConfig, ControlLogic, LaneInput,
)
from bfmc_control.pure_pursuit import PurePursuitConfig
import pytest

# Configuración neutra: cada test cambia solo lo que quiere probar.
BASE = {
    'kp': 1.0, 'ki': 0.0, 'kd': 0.0, 'heading_weight_m_per_rad': 0.0, 'deadband_m': 0.0,
    'integral_limit_rad': 1.0, 'max_steer_rad': 0.5, 'max_steer_rate_rad_s': 1000.0,
    'filter_alpha': 1.0, 'min_confidence': 0.5, 'cruise_speed_mps': 1.0,
    'lost_speed_factor': 0.5, 'slowdown_gain': 0.0, 'max_dt_s': 1.0,
}


def logica(**cambios):
    return ControlLogic(ControlConfig(**{**BASE, **cambios}))


def carril(lateral=0.0, rumbo=0.0, detectado=True, confianza=1.0):
    return LaneInput(detected=detectado, lateral_error_m=lateral,
                     heading_error_rad=rumbo, confidence=confianza)


# --- dirección proporcional ----------------------------------------------------

def test_un_error_a_la_izquierda_gira_a_la_izquierda():
    assert logica(kp=2.0).update(carril(lateral=0.1), now=0.0).steering_rad == pytest.approx(0.2)


def test_un_error_a_la_derecha_gira_a_la_derecha():
    assert logica(kp=2.0).update(carril(lateral=-0.1), now=0.0).steering_rad == pytest.approx(-0.2)


def test_sin_error_no_gira():
    assert logica().update(carril(), now=0.0).steering_rad == 0.0


def test_el_error_de_rumbo_aporta_segun_su_peso():
    salida = logica(heading_weight_m_per_rad=0.5).update(carril(rumbo=0.2), now=0.0)
    assert salida.steering_rad == pytest.approx(0.1)


def test_la_zona_muerta_ignora_errores_chicos():
    control = logica(deadband_m=0.05)
    assert control.update(carril(lateral=0.04), now=0.0).steering_rad == 0.0
    assert control.update(carril(lateral=0.1), now=0.1).steering_rad == pytest.approx(0.1)


def test_la_salida_se_satura_al_angulo_maximo():
    control = logica(kp=100.0)
    assert control.update(carril(lateral=1.0), now=0.0).steering_rad == 0.5
    assert logica(kp=100.0).update(carril(lateral=-1.0), now=0.0).steering_rad == -0.5


# --- término integral y anti-windup ---------------------------------------------

def test_el_integral_acumula_un_error_sostenido():
    control = logica(kp=0.0, ki=1.0)
    assert control.update(carril(lateral=0.125), now=0.0).steering_rad == 0.0
    assert control.update(carril(lateral=0.125), now=1.0).steering_rad == pytest.approx(0.125)
    assert control.update(carril(lateral=0.125), now=2.0).steering_rad == pytest.approx(0.25)


def test_el_integral_no_supera_su_limite():
    control = logica(kp=0.0, ki=1.0, integral_limit_rad=0.15)
    salida = None
    for paso in range(6):
        salida = control.update(carril(lateral=0.125), now=float(paso))
    assert salida.steering_rad == pytest.approx(0.15)


def test_el_integral_se_descarga_cuando_el_error_cambia_de_signo():
    control = logica(kp=0.0, ki=1.0)
    control.update(carril(lateral=0.125), now=0.0)
    control.update(carril(lateral=0.125), now=1.0)
    assert control.update(carril(lateral=-0.125), now=2.0).steering_rad == pytest.approx(0.0)


# --- término derivativo ----------------------------------------------------------

def test_el_derivativo_reacciona_al_cambio_del_error():
    control = logica(kp=0.0, kd=1.0)
    control.update(carril(lateral=0.0), now=0.0)
    assert control.update(carril(lateral=0.125), now=0.5).steering_rad == pytest.approx(0.25)


# --- tiempo -----------------------------------------------------------------------

def test_una_pausa_mayor_al_maximo_reinicia_el_pid():
    control = logica(kp=0.0, ki=1.0, max_dt_s=1.0)
    control.update(carril(lateral=0.125), now=0.0)
    control.update(carril(lateral=0.125), now=1.0)
    assert control.update(carril(lateral=0.125), now=10.0).steering_rad == 0.0


def test_dos_mediciones_con_el_mismo_instante_no_rompen_el_calculo():
    control = logica(kp=1.0, kd=1.0)
    control.update(carril(lateral=0.125), now=1.0)
    assert control.update(carril(lateral=0.25), now=1.0).steering_rad == pytest.approx(0.25)


# --- forma de la salida ---------------------------------------------------------

def test_el_limite_de_velocidad_de_giro_suaviza_los_cambios():
    control = logica(kp=100.0, max_steer_rate_rad_s=1.0)
    control.update(carril(), now=0.0)
    assert control.update(carril(lateral=1.0), now=0.25).steering_rad == pytest.approx(0.25)


def test_el_filtro_suaviza_la_salida():
    control = logica(kp=1.0, filter_alpha=0.5)
    assert control.update(carril(lateral=0.25), now=0.0).steering_rad == pytest.approx(0.25)
    assert control.update(carril(lateral=0.0), now=0.1).steering_rad == pytest.approx(0.125)


# --- carril perdido -----------------------------------------------------------------

def test_sin_carril_la_velocidad_baja_al_factor_configurado():
    salida = logica().update(carril(detectado=False), now=0.0)
    assert salida.speed_mps == pytest.approx(0.5)


def test_con_baja_confianza_se_trata_como_carril_perdido():
    salida = logica().update(carril(lateral=0.2, confianza=0.2), now=0.0)
    assert salida.steering_rad == 0.0
    assert salida.speed_mps == pytest.approx(0.5)


def test_perder_el_carril_reinicia_el_integral():
    control = logica(kp=0.0, ki=1.0)
    control.update(carril(lateral=0.125), now=0.0)
    control.update(carril(lateral=0.125), now=1.0)
    control.update(carril(detectado=False), now=1.5)
    control.update(carril(lateral=0.125), now=2.0)
    assert control.update(carril(lateral=0.125), now=3.0).steering_rad == pytest.approx(0.125)


def test_al_perder_el_carril_la_direccion_vuelve_al_centro_con_el_limite_de_giro():
    control = logica(kp=100.0, max_steer_rate_rad_s=1.0)
    control.update(carril(lateral=1.0), now=0.0)
    assert control.update(carril(detectado=False), now=0.25).steering_rad == pytest.approx(0.25)


def test_al_perder_el_carril_se_sostiene_el_ultimo_giro_durante_hold_steer_s():
    control = logica(kp=1.0, hold_steer_s=0.5)
    control.update(carril(lateral=0.25), now=0.0)
    assert control.update(carril(detectado=False), now=0.25).steering_rad == pytest.approx(0.25)
    assert control.update(carril(detectado=False), now=0.5).steering_rad == pytest.approx(0.25)


def test_pasado_hold_steer_s_la_direccion_vuelve_al_centro():
    control = logica(kp=1.0, hold_steer_s=0.5)
    control.update(carril(lateral=0.25), now=0.0)
    control.update(carril(detectado=False), now=0.25)
    assert control.update(carril(detectado=False), now=0.75).steering_rad == 0.0


def test_la_baja_confianza_tambien_sostiene_el_giro():
    control = logica(kp=1.0, hold_steer_s=0.5)
    control.update(carril(lateral=-0.25), now=0.0)
    salida = control.update(carril(lateral=0.4, confianza=0.1), now=0.25)
    assert salida.steering_rad == pytest.approx(-0.25)
    assert salida.speed_mps == pytest.approx(0.5)


def test_sin_carril_desde_el_arranque_no_hay_giro_que_sostener():
    assert logica(hold_steer_s=0.5).update(carril(detectado=False), now=0.0).steering_rad == 0.0


def test_al_recuperar_el_carril_manda_el_carril_y_no_el_giro_sostenido():
    control = logica(kp=1.0, hold_steer_s=0.5)
    control.update(carril(lateral=0.25), now=0.0)
    control.update(carril(detectado=False), now=0.1)
    assert control.update(carril(lateral=-0.1), now=0.2).steering_rad == pytest.approx(-0.1)


def test_el_giro_sostenido_respeta_el_limite_de_velocidad_de_giro():
    control = logica(kp=1.0, hold_steer_s=1.0, max_steer_rate_rad_s=1.0)
    control.update(carril(lateral=0.0), now=0.0)
    control.update(carril(lateral=0.1), now=0.1)          # llega a 0,1 (tope 0,1 por paso)
    control.update(carril(detectado=False), now=0.2)
    control.update(carril(detectado=False), now=1.1)      # último instante del sostén
    # Pasado el sostén va al centro de a 1 rad/s, no de golpe: 0,05 s después baja 0,05.
    assert control.update(carril(detectado=False), now=1.15).steering_rad == pytest.approx(0.05)


# --- término de curvatura -------------------------------------------------------

PP = PurePursuitConfig(wheelbase_m=0.25, camera_to_rear_axle_m=0.2, lookahead_min_m=0.7,
                       lookahead_max_m=1.1, lookahead_gain_s=0.0, max_lateral_accel_mps2=1.0,
                       speed_min_factor=0.5)
FF = {'curvature_ff_gain': 1.0, 'curvature_ff_min_confidence': 0.9, 'curvature_ff_max': 2.0,
      'curvature_ff_hold_s': 0.5, 'pure_pursuit': PP}


def curva(kappa, confianza=1.0):
    return LaneInput(detected=True, lateral_error_m=0.0, heading_error_rad=0.0,
                     confidence=confianza, curvature=kappa)


def test_centrado_en_una_curva_gira_lo_que_pide_la_curva():
    salida = logica(**FF).update(curva(1.0), now=0.0)
    assert salida.steering_rad == pytest.approx(math.atan(0.25 * 1.0))


def test_la_curva_a_la_derecha_gira_a_la_derecha():
    assert logica(**FF).update(curva(-1.0), now=0.0).steering_rad < 0.0


def test_sin_ganancia_la_curvatura_no_cambia_nada():
    control = logica(**{**FF, 'curvature_ff_gain': 0.0})
    assert control.update(curva(1.0), now=0.0).steering_rad == 0.0


def test_con_una_sola_linea_no_se_toma_la_curvatura():
    assert logica(**FF).update(curva(1.0, confianza=0.5), now=0.0).steering_rad == 0.0


def test_la_ultima_curvatura_confiable_se_sostiene_y_despues_se_olvida():
    control = logica(**FF)
    control.update(curva(1.0), now=0.0)
    assert control.update(curva(-3.0, confianza=0.5), now=0.4).steering_rad == pytest.approx(
        math.atan(0.25))
    assert control.update(curva(-3.0, confianza=0.5), now=1.0).steering_rad == 0.0


def test_la_curvatura_se_limita():
    salida = logica(**FF).update(curva(10.0), now=0.0)
    assert salida.steering_rad == pytest.approx(math.atan(0.25 * 2.0))


def test_la_curvatura_no_finita_se_ignora():
    assert logica(**FF).update(curva(float('nan')), now=0.0).steering_rad == 0.0


def test_el_termino_de_curvatura_no_baja_la_velocidad():
    assert logica(**FF).update(curva(2.0), now=0.0).speed_mps == pytest.approx(1.0)


def test_el_termino_de_curvatura_sin_wheelbase_se_rechaza():
    with pytest.raises(ValueError):
        ControlConfig(**{**BASE, 'curvature_ff_gain': 0.5})


# --- velocidad -----------------------------------------------------------------------

def test_sin_giro_se_mantiene_la_velocidad_de_crucero():
    assert logica(slowdown_gain=0.5).update(carril(), now=0.0).speed_mps == pytest.approx(1.0)


def test_al_girar_al_maximo_la_velocidad_baja_segun_la_ganancia():
    salida = logica(kp=100.0, slowdown_gain=0.5).update(carril(lateral=1.0), now=0.0)
    assert salida.speed_mps == pytest.approx(0.5)


# --- validación de la configuración ---------------------------------------------------

@pytest.mark.parametrize('cambio', [
    {'kp': -1.0},
    {'ki': -0.1},
    {'kd': -0.1},
    {'max_steer_rad': 0.0},
    {'max_steer_rate_rad_s': 0.0},
    {'filter_alpha': 0.0},
    {'filter_alpha': 1.5},
    {'min_confidence': 1.5},
    {'lost_speed_factor': 1.5},
    {'slowdown_gain': 1.0},
    {'max_dt_s': 0.0},
    {'hold_steer_s': -0.1},
    {'curvature_ff_gain': -0.1},
    {'curvature_ff_min_confidence': 1.5},
    {'curvature_ff_max': 0.0},
    {'curvature_ff_hold_s': -0.1},
    {'cruise_speed_mps': -1.0},
])
def test_una_configuracion_invalida_se_rechaza(cambio):
    with pytest.raises(ValueError):
        ControlConfig(**{**BASE, **cambio})


# --- configuración en vivo -------------------------------------------------------

def test_cambiar_la_velocidad_de_crucero_se_nota_en_la_salida_siguiente():
    control = logica(cruise_speed_mps=0.3)
    assert control.update(carril(), now=0.0).speed_mps == pytest.approx(0.3)
    control.set_config(replace(control.config, cruise_speed_mps=0.1))
    assert control.update(carril(), now=0.1).speed_mps == pytest.approx(0.1)


def test_cambiar_la_configuracion_no_reinicia_el_pid():
    control = logica(kp=0.0, ki=1.0, integral_limit_rad=1.0)
    control.update(carril(lateral=0.1), now=0.0)
    antes = control.update(carril(lateral=0.1), now=0.1).steering_rad
    control.set_config(replace(control.config, cruise_speed_mps=0.1))
    despues = control.update(carril(lateral=0.1), now=0.2).steering_rad
    assert despues > antes > 0.0
