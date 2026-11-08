"""Tests del filtro de Kalman del carril (lane_kalman.py) y su uso en LaneTracker, sin ROS.

Las mediciones son secuencias sintéticas generadas acá: constantes, con ruido, con un salto
aislado y con huecos, a 30 cuadros por segundo.
"""
from bfmc_lane_detection.lane_detection_logic import LaneResult
from bfmc_lane_detection.lane_kalman import LaneKalman
from bfmc_lane_detection.lane_tracker import (
    LaneTracker, MODE_LOST, MODE_PREDICTED, MODE_TWO_LINES,
)
import numpy as np
import pytest

DT = 1.0 / 30.0
Q = (0.0004, 0.01, 0.25)
R_DOS = (0.0001, 0.0025, 0.25)
R_UNA = (0.0004, 0.01, 1.0)
GATE = 11.34


def filtro():
    return LaneKalman(Q, R_DOS, R_UNA, GATE)


def medicion(d, psi=0.0, kappa=0.0, dos=True):
    return LaneResult(True, d, psi, 1.0, False, left_valid=True, right_valid=dos,
                      curvature=kappa)


SIN_LINEAS = LaneResult(False, 0.0, 0.0, 0.0, False)


def tracker_con_filtro(timeout_s=0.3):
    tracker = LaneTracker(timeout_s)
    tracker.usar_kalman(filtro(), max_std_m=0.1)
    return tracker


# --- el filtro solo ------------------------------------------------------------------------

def test_con_mediciones_constantes_converge_y_la_incertidumbre_baja():
    kf = filtro()
    kf.reiniciar((0.0, 0.0, 0.0), dos_lineas=True)
    inicial = kf.desvio_std_m
    for _ in range(60):
        kf.predecir(DT, 0.3)
        assert kf.corregir((0.05, 0.02, 0.0), dos_lineas=True)
    assert kf.estado == pytest.approx((0.05, 0.02, 0.0), abs=0.005)
    assert kf.desvio_std_m < inicial


def test_con_ruido_el_filtrado_varia_menos_que_lo_medido():
    azar = np.random.default_rng(0)
    medidas = 0.05 + azar.normal(0.0, 0.01, 200)
    kf = filtro()
    kf.reiniciar((medidas[0], 0.0, 0.0), dos_lineas=True)
    filtradas = []
    for d in medidas[1:]:
        kf.predecir(DT, 0.3)
        kf.corregir((d, 0.0, 0.0), dos_lineas=True)
        filtradas.append(kf.estado[0])
    assert np.std(filtradas[50:]) < 0.5 * np.std(medidas[50:])


def test_un_salto_aislado_de_20_cm_se_descarta_y_no_mueve_la_estimacion():
    kf = filtro()
    kf.reiniciar((0.0, 0.0, 0.0), dos_lineas=True)
    for _ in range(30):
        kf.predecir(DT, 0.3)
        kf.corregir((0.0, 0.0, 0.0), dos_lineas=True)
    kf.predecir(DT, 0.3)
    assert kf.corregir((0.20, 0.0, 0.0), dos_lineas=True) is False
    assert kf.estado[0] == pytest.approx(0.0, abs=0.005)


def test_sin_mediciones_predice_con_el_avance_y_la_incertidumbre_crece():
    kf = filtro()
    kf.reiniciar((0.0, 0.1, 0.0), dos_lineas=True)
    incertidumbres = [kf.desvio_std_m]
    for _ in range(5):
        kf.predecir(DT, 0.3)
        incertidumbres.append(kf.desvio_std_m)
    # d' = d + v·dt·psi en cada cuadro: 5 cuadros a 0,3 m/s con 0,1 rad.
    assert kf.estado[0] == pytest.approx(5 * 0.3 * DT * 0.1)
    assert incertidumbres == sorted(incertidumbres)
    assert incertidumbres[-1] > incertidumbres[0]


def test_con_una_linea_la_medicion_pesa_menos():
    con_dos, con_una = filtro(), filtro()
    for kf in (con_dos, con_una):
        kf.reiniciar((0.0, 0.0, 0.0), dos_lineas=True)
    con_dos.corregir((0.02, 0.0, 0.0), dos_lineas=True)
    con_una.corregir((0.02, 0.0, 0.0), dos_lineas=False)
    assert 0.0 < con_una.estado[0] < con_dos.estado[0]


@pytest.mark.parametrize('q, r_dos, r_una, gate', [
    ((0.0, 0.01, 0.25), R_DOS, R_UNA, GATE),
    (Q, (0.0001, 0.0025), R_UNA, GATE),
    (Q, R_DOS, R_UNA, 0.0),
])
def test_parametros_invalidos_se_rechazan(q, r_dos, r_una, gate):
    with pytest.raises(ValueError):
        LaneKalman(q, r_dos, r_una, gate)


# --- en el tracker -------------------------------------------------------------------------

def test_sin_filtro_el_tracker_se_comporta_como_antes():
    tracker = LaneTracker(0.3)
    estado = tracker.update(medicion(0.05, kappa=0.4), 10.0)
    assert (estado.lateral_error_m, estado.curvature) == (0.05, 0.4)


def test_con_filtro_un_salto_aislado_no_llega_a_lo_publicado():
    tracker = tracker_con_filtro()
    for i in range(30):
        tracker.update(medicion(0.0), 10.0 + i * DT, 0.3)
    estado = tracker.update(medicion(0.20), 10.0 + 30 * DT, 0.3)
    assert estado.mode == MODE_TWO_LINES
    assert estado.lateral_error_m == pytest.approx(0.0, abs=0.005)


def test_con_filtro_en_predicho_sigue_la_prediccion_y_baja_la_confianza():
    tracker = tracker_con_filtro()
    for i in range(30):
        tracker.update(medicion(0.0, psi=0.1), 10.0 + i * DT, 0.3)
    antes = tracker.update(medicion(0.0, psi=0.1), 11.0, 0.3)
    despues = tracker.update(SIN_LINEAS, 11.0 + 5 * DT, 0.3)
    assert despues.mode == MODE_PREDICTED
    assert despues.lateral_error_m > antes.lateral_error_m      # avanzó con el rumbo
    assert despues.confidence < antes.confidence


def test_con_filtro_al_salir_de_perdido_con_dos_lineas_se_reinicia():
    tracker = tracker_con_filtro()
    tracker.update(medicion(0.0), 10.0, 0.3)
    assert tracker.update(SIN_LINEAS, 11.0, 0.3).mode == MODE_LOST
    estado = tracker.update(medicion(0.15), 11.03, 0.3)
    assert estado.lateral_error_m == pytest.approx(0.15)       # arranca de la medición


def test_con_filtro_si_descarta_mediciones_por_mas_del_plazo_se_reinicia():
    # La realidad cambió de golpe (por ejemplo, después de una intersección): sin
    # reiniciar, el filtro quedaría aferrado a su estimación vieja para siempre.
    tracker = tracker_con_filtro(timeout_s=0.3)
    for i in range(30):
        tracker.update(medicion(0.0), 10.0 + i * DT, 0.0)
    instante = 10.0 + 30 * DT
    estado = None
    for i in range(15):                                          # 0,5 s de saltos
        estado = tracker.update(medicion(0.20), instante + i * DT, 0.0)
    assert estado.lateral_error_m == pytest.approx(0.20, abs=0.01)


def test_con_filtro_olvidar_previas_tambien_reinicia_el_filtro():
    tracker = tracker_con_filtro()
    tracker.update(medicion(0.0), 10.0, 0.3)
    tracker.olvidar_previas()
    assert tracker.update(medicion(0.12), 10.03, 0.3).lateral_error_m == pytest.approx(0.12)
