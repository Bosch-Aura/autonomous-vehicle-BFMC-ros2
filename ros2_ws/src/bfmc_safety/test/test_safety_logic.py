"""Tests de la lógica de seguridad: sin ROS y con el tiempo simulado."""
import math

from bfmc_safety.safety_logic import (
    Reading, Reason, SafetyConfig, SafetyLogic,
)
import pytest

MIN_M = 0.02
MAX_M = 4.0


def logica(**cambios):
    base = {'stop_distance_m': 0.25, 'resume_distance_m': 0.5, 'stale_timeout_s': 0.5,
            'emergency_on_invalid': True}
    return SafetyLogic(SafetyConfig(**{**base, **cambios}))


def lectura(distancia):
    return Reading(range_m=distancia, min_range_m=MIN_M, max_range_m=MAX_M)


def evaluar(logic, distancia, now=0.0):
    logic.on_reading(lectura(distancia), now)
    return logic.evaluate(now)


# --- obstáculo -----------------------------------------------------------------------

def test_un_obstaculo_por_debajo_del_umbral_es_una_emergencia():
    resultado = evaluar(logica(), 0.125)
    assert resultado.emergency is True
    assert resultado.reason == Reason.OBSTACLE
    assert resultado.distance_m == pytest.approx(0.125)


def test_un_obstaculo_lejano_no_es_una_emergencia():
    resultado = evaluar(logica(), 1.0)
    assert resultado.emergency is False
    assert resultado.reason == Reason.NONE
    assert resultado.sensor_valid is True


def test_justo_en_el_umbral_de_parada_todavia_no_es_una_emergencia():
    assert evaluar(logica(), 0.25).emergency is False


# --- histéresis -----------------------------------------------------------------------

def test_entre_los_dos_umbrales_se_mantiene_la_emergencia():
    logic = logica()
    evaluar(logic, 0.125, now=0.0)
    assert evaluar(logic, 0.375, now=0.1).emergency is True


def test_al_superar_el_umbral_de_liberacion_la_emergencia_termina():
    logic = logica()
    evaluar(logic, 0.125, now=0.0)
    resultado = evaluar(logic, 0.75, now=0.1)
    assert resultado.emergency is False
    assert resultado.reason == Reason.NONE


def test_entre_los_umbrales_sin_emergencia_previa_no_se_activa():
    logic = logica()
    evaluar(logic, 1.0, now=0.0)
    assert evaluar(logic, 0.375, now=0.1).emergency is False


# --- interpretación de la lectura -----------------------------------------------------------

def test_sin_eco_se_interpreta_como_camino_despejado():
    resultado = evaluar(logica(), math.inf)
    assert resultado.emergency is False
    assert resultado.sensor_valid is True
    assert resultado.distance_m == MAX_M


def test_una_lectura_mas_alla_del_alcance_es_camino_despejado():
    resultado = evaluar(logica(), 10.0)
    assert resultado.emergency is False
    assert resultado.distance_m == MAX_M


def test_un_obstaculo_mas_cerca_del_minimo_cuenta_como_obstaculo_en_el_minimo():
    resultado = evaluar(logica(), -math.inf)
    assert resultado.emergency is True
    assert resultado.reason == Reason.OBSTACLE
    assert resultado.distance_m == MIN_M


def test_una_lectura_nan_es_un_error_del_sensor():
    resultado = evaluar(logica(), math.nan)
    assert resultado.emergency is True
    assert resultado.reason == Reason.SENSOR_INVALID
    assert resultado.sensor_valid is False
    assert math.isnan(resultado.distance_m)


def test_una_lectura_nan_no_es_emergencia_si_se_configura_asi():
    resultado = evaluar(logica(emergency_on_invalid=False), math.nan)
    assert resultado.emergency is False
    assert resultado.reason == Reason.NONE
    assert resultado.sensor_valid is False


# --- lecturas ausentes o vencidas -------------------------------------------------------------

def test_sin_ninguna_lectura_hay_emergencia_por_sensor_vencido():
    resultado = logica().evaluate(now=0.0)
    assert resultado.emergency is True
    assert resultado.reason == Reason.SENSOR_STALE
    assert resultado.sensor_valid is False


def test_una_lectura_dentro_del_tiempo_limite_sigue_vigente():
    logic = logica(stale_timeout_s=0.5)
    logic.on_reading(lectura(1.0), now=10.0)
    assert logic.evaluate(now=10.5).emergency is False


def test_una_lectura_mas_vieja_que_el_tiempo_limite_es_una_emergencia():
    logic = logica(stale_timeout_s=0.5)
    logic.on_reading(lectura(1.0), now=10.0)
    resultado = logic.evaluate(now=10.75)
    assert resultado.emergency is True
    assert resultado.reason == Reason.SENSOR_STALE


def test_una_lectura_nueva_termina_la_emergencia_por_sensor_vencido():
    logic = logica(stale_timeout_s=0.5)
    logic.on_reading(lectura(1.0), now=10.0)
    assert logic.evaluate(now=11.0).emergency is True
    logic.on_reading(lectura(1.0), now=11.25)
    assert logic.evaluate(now=11.25).emergency is False


def test_perder_el_sensor_reinicia_la_memoria_del_obstaculo():
    logic = logica(stale_timeout_s=0.5)
    evaluar(logic, 0.125, now=0.0)
    logic.evaluate(now=5.0)
    assert evaluar(logic, 0.375, now=5.25).emergency is False


# --- contrato con el mensaje -------------------------------------------------------------------

def test_los_motivos_tienen_los_valores_del_mensaje():
    assert [int(r) for r in Reason] == [0, 1, 2, 3]


# --- validación de la configuración ------------------------------------------------------------

@pytest.mark.parametrize('cambio', [
    {'stop_distance_m': 0.0},
    {'stop_distance_m': -0.1},
    {'resume_distance_m': 0.25},
    {'resume_distance_m': 0.1},
    {'stale_timeout_s': 0.0},
])
def test_una_configuracion_invalida_se_rechaza(cambio):
    with pytest.raises(ValueError):
        SafetyConfig(**{**{'stop_distance_m': 0.25, 'resume_distance_m': 0.5,
                           'stale_timeout_s': 0.5, 'emergency_on_invalid': True}, **cambio})
