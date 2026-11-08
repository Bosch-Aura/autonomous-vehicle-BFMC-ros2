"""Tests del fallback escalonado (lane_tracker.py), sin ROS: el tiempo se pasa a mano."""
import math

from bfmc_lane_detection.lane_detection_logic import LaneResult
from bfmc_lane_detection.lane_tracker import (
    LaneTracker, MODE_LOST, MODE_ONE_LINE, MODE_PREDICTED, MODE_TWO_LINES,
)
import pytest

TIMEOUT_S = 0.3
DOS = LaneResult(True, 0.05, 0.1, 1.0, False, left_valid=True, right_valid=True)
UNA = LaneResult(True, 0.02, 0.0, 0.5, False, left_valid=True, right_valid=False)
NINGUNA = LaneResult(False, 0.0, 0.0, 0.0, False, failed_checks='ancho de carril')


def test_con_las_dos_lineas_validas_el_modo_es_dos_lineas():
    estado = LaneTracker(TIMEOUT_S).update(DOS, 10.0)
    assert estado.mode == MODE_TWO_LINES
    assert (estado.detected, estado.lateral_error_m, estado.confidence) == (True, 0.05, 1.0)


def test_con_una_linea_valida_el_modo_es_una_linea():
    estado = LaneTracker(TIMEOUT_S).update(UNA, 10.0)
    assert estado.mode == MODE_ONE_LINE
    assert (estado.left_valid, estado.right_valid) == (True, False)


def test_sin_lineas_poco_despues_de_una_medicion_se_repite_el_ultimo_estado():
    tracker = LaneTracker(TIMEOUT_S)
    tracker.update(DOS, 10.0)
    estado = tracker.update(NINGUNA, 10.1)
    assert estado.mode == MODE_PREDICTED
    assert estado.detected is True
    assert (estado.lateral_error_m, estado.heading_error_rad) == (0.05, 0.1)
    # Pasó un tercio del plazo: la confianza bajó un tercio.
    assert estado.confidence == pytest.approx(1.0 * (1 - 0.1 / TIMEOUT_S))
    # Las líneas válidas son las de la imagen actual: ninguna.
    assert (estado.left_valid, estado.right_valid) == (False, False)


def test_la_confianza_predicha_baja_hasta_cero_al_cumplirse_el_plazo():
    tracker = LaneTracker(TIMEOUT_S)
    tracker.update(DOS, 10.0)
    confianzas = [tracker.update(NINGUNA, 10.0 + t).confidence for t in (0.05, 0.15, 0.29)]
    assert confianzas == sorted(confianzas, reverse=True)
    assert confianzas[-1] < 0.05


def test_pasado_el_plazo_sin_lineas_el_carril_se_pierde():
    tracker = LaneTracker(TIMEOUT_S)
    tracker.update(DOS, 10.0)
    assert tracker.update(NINGUNA, 10.2).mode == MODE_PREDICTED
    estado = tracker.update(NINGUNA, 10.31)
    assert estado.mode == MODE_LOST
    assert estado.detected is False
    assert estado.confidence == 0.0
    assert math.isnan(estado.lateral_error_m) and math.isnan(estado.heading_error_rad)


def test_sin_ninguna_medicion_previa_el_carril_esta_perdido():
    assert LaneTracker(TIMEOUT_S).update(NINGUNA, 10.0).mode == MODE_LOST


def test_cuando_vuelven_las_lineas_se_recupera_solo():
    tracker = LaneTracker(TIMEOUT_S)
    tracker.update(DOS, 10.0)
    assert tracker.update(NINGUNA, 11.0).mode == MODE_LOST
    assert tracker.update(UNA, 11.1).mode == MODE_ONE_LINE


def test_si_el_reloj_vuelve_atras_se_toma_como_perdido():
    tracker = LaneTracker(TIMEOUT_S)
    tracker.update(DOS, 10.0)
    assert tracker.update(NINGUNA, 9.9).mode == MODE_LOST


def test_la_interseccion_es_siempre_la_de_la_imagen_actual():
    tracker = LaneTracker(TIMEOUT_S)
    tracker.update(DOS, 10.0)
    franja = LaneResult(False, 0.0, 0.0, 0.0, True)
    assert tracker.update(franja, 10.1).intersection_detected is True
    assert tracker.update(franja, 11.0).intersection_detected is True


def test_cambiar_el_plazo_no_borra_la_memoria():
    tracker = LaneTracker(TIMEOUT_S)
    tracker.update(DOS, 10.0)
    tracker.lost_timeout_s = 1.0
    assert tracker.update(NINGUNA, 10.5).mode == MODE_PREDICTED


@pytest.mark.parametrize('plazo', [0.0, -1.0])
def test_un_plazo_no_positivo_se_rechaza(plazo):
    with pytest.raises(ValueError):
        LaneTracker(plazo)


# --- polinomios previos para buscar cerca (A2-153) ------------------------------------

AJUSTE_IZQ = (0.0, 0.0, 50.0)
AJUSTE_DER = (0.0, 0.0, 150.0)
CON_AJUSTES = LaneResult(True, 0.0, 0.0, 1.0, False, left_valid=True, right_valid=True,
                         left_fit=AJUSTE_IZQ, right_fit=AJUSTE_DER)


def test_sin_mediciones_no_hay_previas():
    assert LaneTracker(TIMEOUT_S).previas(10.0) == (None, None)


def test_las_lineas_validas_quedan_como_previas():
    tracker = LaneTracker(TIMEOUT_S)
    tracker.update(CON_AJUSTES, 10.0)
    assert tracker.previas(10.03) == (AJUSTE_IZQ, AJUSTE_DER)


def test_un_hueco_breve_conserva_la_previa_y_uno_largo_la_olvida():
    # Las reglas son por línea: la izquierda deja de ser válida, la derecha sigue.
    tracker = LaneTracker(TIMEOUT_S)
    tracker.update(CON_AJUSTES, 10.0)
    solo_derecha = LaneResult(True, 0.0, 0.0, 0.5, False, right_valid=True,
                              right_fit=AJUSTE_DER)
    tracker.update(solo_derecha, 10.2)
    assert tracker.previas(10.2) == (AJUSTE_IZQ, AJUSTE_DER)    # hueco de punteada
    tracker.update(solo_derecha, 10.4)
    assert tracker.previas(10.4) == (None, AJUSTE_DER)          # más de lost_timeout_s


def test_si_el_ancho_descarto_una_linea_su_previa_se_olvida_enseguida():
    tracker = LaneTracker(TIMEOUT_S)
    tracker.update(CON_AJUSTES, 10.0)
    descartada = LaneResult(True, 0.0, 0.0, 0.5, False, left_valid=True, left_fit=AJUSTE_IZQ,
                            width_discarded='right')
    tracker.update(descartada, 10.03)
    assert tracker.previas(10.03) == (AJUSTE_IZQ, None)


def test_olvidar_previas_obliga_a_buscar_de_nuevo():
    tracker = LaneTracker(TIMEOUT_S)
    tracker.update(CON_AJUSTES, 10.0)
    tracker.olvidar_previas()
    assert tracker.previas(10.03) == (None, None)
