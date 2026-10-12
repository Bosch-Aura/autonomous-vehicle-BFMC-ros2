"""Tests de la lógica del control: sin ROS y con el tiempo simulado."""
import pytest

from bfmc_control.control_logic import (
    ControlConfig, ControlLogic, LaneInput,
)

# Configuración neutra: cada test cambia solo lo que quiere probar.
BASE = dict(
    kp=1.0, ki=0.0, kd=0.0, heading_weight_m_per_rad=0.0, deadband_m=0.0,
    integral_limit_rad=1.0, max_steer_rad=0.5, max_steer_rate_rad_s=1000.0,
    filter_alpha=1.0, min_confidence=0.5, cruise_speed_mps=1.0,
    lost_speed_factor=0.5, slowdown_gain=0.0, max_dt_s=1.0,
)


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


# --- velocidad -----------------------------------------------------------------------

def test_sin_giro_se_mantiene_la_velocidad_de_crucero():
    assert logica(slowdown_gain=0.5).update(carril(), now=0.0).speed_mps == pytest.approx(1.0)


def test_al_girar_al_maximo_la_velocidad_baja_segun_la_ganancia():
    salida = logica(kp=100.0, slowdown_gain=0.5).update(carril(lateral=1.0), now=0.0)
    assert salida.speed_mps == pytest.approx(0.5)


# --- validación de la configuración ---------------------------------------------------

@pytest.mark.parametrize('cambio', [
    dict(kp=-1.0),
    dict(ki=-0.1),
    dict(kd=-0.1),
    dict(max_steer_rad=0.0),
    dict(max_steer_rate_rad_s=0.0),
    dict(filter_alpha=0.0),
    dict(filter_alpha=1.5),
    dict(min_confidence=1.5),
    dict(lost_speed_factor=1.5),
    dict(slowdown_gain=1.0),
    dict(max_dt_s=0.0),
    dict(cruise_speed_mps=-1.0),
])
def test_una_configuracion_invalida_se_rechaza(cambio):
    with pytest.raises(ValueError):
        ControlConfig(**{**BASE, **cambio})
