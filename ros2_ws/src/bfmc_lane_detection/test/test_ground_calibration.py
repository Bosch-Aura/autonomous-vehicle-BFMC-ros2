"""Tests de la calibración y la geometría de la vista cenital (ground_calibration.py), sin ROS.

La profundidad es la de un piso sintético con altura e inclinación conocidas
(synthetic_depth.py). Lo que se verifica es lo que pide A2-151: recuperar la altura con
error menor a 5 mm y el pitch con error menor a 0,5° aun con ruido y un obstáculo, y que
la escala resultante ubique en la vista cenital un punto del piso conocido. Además, que la
geometría salga solo de datos independientes: la pose, los intrínsecos y el rectángulo.
"""
from dataclasses import replace
import math
from pathlib import Path

from bfmc_lane_detection.ground_calibration import (
    a_la_imagen, calibrar_piso, CalibrationError, GroundCalibrationConfig, GroundPose,
    GroundView, intrinsecos_de, Intrinsics, lane_config_desde_parametros, medir_pose,
    promediar, punto_del_piso, trapecio,
)
from bfmc_lane_detection.lane_detection_logic import LaneDetector
import cv2
import numpy as np
import pytest
from synthetic_depth import (
    ALTO, ANCHO, CX, CY, FX, FY, normal_del_piso, pared, profundidad_del_piso,
)
from synthetic_lanes import lienzo
import yaml

PARAMS = Path(__file__).resolve().parents[1] / 'config' / 'params.yaml'
INTR = Intrinsics(FX, FY, CX, CY, ANCHO, ALTO)
ALTURA_M = 0.20
PITCH_DEG = 20.0
POSE_REAL = GroundPose(ALTURA_M, PITCH_DEG, 0.0)
VISTA = GroundView(0.45, 0.95, 0.25)


@pytest.fixture(scope='module')
def parametros():
    with open(PARAMS, encoding='utf-8') as archivo:
        return yaml.safe_load(archivo)['lane_detection']['ros__parameters']


@pytest.fixture(scope='module')
def cfg(parametros):
    return GroundCalibrationConfig(
        depth_min_m=parametros['calib_depth_min_m'],
        depth_max_m=parametros['calib_depth_max_m'],
        subsample=parametros['calib_subsample'],
        min_points=parametros['calib_min_points'],
        ransac_iters=parametros['calib_ransac_iters'],
        ransac_seed=parametros['calib_ransac_seed'],
        inlier_m=parametros['calib_inlier_m'],
        min_inlier_fraction=parametros['calib_min_inlier_fraction'],
        height_range_m=tuple(parametros['calib_height_range_m']),
        pitch_range_deg=tuple(parametros['calib_pitch_range_deg']))


def cuadros(n=10, **piso):
    """`n` cuadros del piso con 3 mm de ruido (distinto en cada uno) y lo que se pida."""
    valores = {'altura_m': ALTURA_M, 'pitch_deg': PITCH_DEG, 'ruido_m': 0.003}
    valores.update(piso)
    return [profundidad_del_piso(semilla=i, **valores) for i in range(n)]


def calibrar(cfg, profundidades, vista=VISTA):
    return calibrar_piso(profundidades, INTR, cfg, vista, 400, 400)


def parametros_con(parametros, resultado, **cambios):
    """Los parámetros del YAML con la pose y los intrínsecos de una calibración."""
    return {**parametros, **resultado.parametros(), **cambios}


def detector_calibrado(parametros, resultado, **cambios):
    return LaneDetector(lane_config_desde_parametros(
        parametros_con(parametros, resultado, **cambios)))


def pixel_real(adelante_m, izquierda_m):
    """Píxel donde la cámara sintética ve un punto del piso, con la pose EXACTA."""
    return a_la_imagen(punto_del_piso(POSE_REAL, adelante_m, izquierda_m), INTR)


# --- pose de la cámara ----------------------------------------------------------------------

def test_recupera_altura_y_pitch_con_ruido_y_un_obstaculo(cfg):
    pose = medir_pose(cuadros(fraccion_basura=0.2), INTR, cfg)
    assert pose.height_m == pytest.approx(ALTURA_M, abs=0.005)
    assert pose.pitch_deg == pytest.approx(PITCH_DEG, abs=0.5)
    assert pose.inlier_fraction >= cfg.min_inlier_fraction


@pytest.mark.parametrize('altura_m, pitch_deg, roll_deg', [
    (0.15, 10.0, 0.0), (0.30, 35.0, 0.0), (0.20, 20.0, 3.0), (0.20, 20.0, -3.0)])
def test_recupera_otras_poses(cfg, altura_m, pitch_deg, roll_deg):
    pose = medir_pose(cuadros(altura_m=altura_m, pitch_deg=pitch_deg, roll_deg=roll_deg),
                      INTR, cfg)
    assert pose.height_m == pytest.approx(altura_m, abs=0.005)
    assert pose.pitch_deg == pytest.approx(pitch_deg, abs=0.5)
    # Positivo = el lado derecho de la cámara más alto.
    assert pose.roll_deg == pytest.approx(roll_deg, abs=0.3)


@pytest.mark.parametrize('pitch_deg, roll_deg', [(20.0, 0.0), (20.0, 5.0), (35.0, -4.0)])
def test_pitch_y_roll_reconstruyen_el_mismo_plano(pitch_deg, roll_deg):
    # Por eso alcanza con guardar altura, pitch y roll para volver a proyectar el trapecio.
    pose = GroundPose(ALTURA_M, pitch_deg, roll_deg)
    assert pose.normal == pytest.approx(normal_del_piso(pitch_deg, roll_deg))
    assert np.linalg.norm(pose.normal) == pytest.approx(1.0)


def test_el_mismo_cuadro_da_siempre_el_mismo_resultado(cfg):
    profundidades = cuadros(fraccion_basura=0.2)
    assert calibrar(cfg, profundidades) == calibrar(cfg, profundidades)


def test_promediar_ignora_los_pixeles_sin_dato():
    a = np.array([[1.0, 0.0, np.nan]])
    b = np.array([[3.0, 2.0, 0.0]])
    promedio = promediar([a, b])
    assert promedio[0, 0] == pytest.approx(2.0)
    assert promedio[0, 1] == pytest.approx(2.0)
    assert np.isnan(promedio[0, 2])


# --- trapecio y escala ----------------------------------------------------------------------

def test_la_escala_sale_de_las_medidas_del_rectangulo(cfg):
    resultado = calibrar(cfg, cuadros())
    # 0,50 m en 400 px: 1 px = 1,25 mm en los dos ejes.
    assert resultado.mm_per_px_x == pytest.approx(1.25)
    assert resultado.mm_per_px_y == pytest.approx(1.25)
    assert '1 px = 1.25 mm' in resultado.resumen()


@pytest.mark.parametrize('adelante_m, izquierda_m', [(0.70, 0.10), (0.50, -0.20), (0.90, 0.0)])
def test_ida_y_vuelta_un_punto_del_piso_cae_donde_predice_la_escala(
        cfg, parametros, adelante_m, izquierda_m):
    resultado = calibrar(cfg, cuadros())
    detector = detector_calibrado(parametros, resultado)
    cenital = cv2.perspectiveTransform(
        np.array([[pixel_real(adelante_m, izquierda_m)]], dtype=np.float64),
        detector.warp_matrix(ANCHO, ALTO))[0, 0]

    # La escala y el borde cercano son los del rectángulo del YAML (el del detector).
    config = detector.config
    mm_x = config.view_width_m / config.warp_width
    mm_y = config.view_length_m / config.warp_height
    esperado_x = config.warp_width / 2.0 - izquierda_m / mm_x      # izquierda = columnas menores
    esperado_y = config.warp_height - (adelante_m - parametros['calib_forward_near_m']) / mm_y
    # Criterio de A2-151: error menor a 1 cm a 0,5 m y a 2 cm a 1 m (8 px = 1 cm).
    assert cenital[0] == pytest.approx(esperado_x, abs=4.0)
    assert cenital[1] == pytest.approx(esperado_y, abs=4.0)


def carril_recto(corrimiento_m, carril_m=0.37):
    """Imagen de un carril recto con su centro `corrimiento_m` a la izquierda del auto."""
    imagen = lienzo(ANCHO, ALTO)
    for lado in (-carril_m / 2, carril_m / 2):
        puntos = [pixel_real(x, corrimiento_m + lado) for x in np.linspace(0.40, 1.0, 13)]
        puntos = np.round(np.array(puntos)).astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(imagen, [puntos], False, (255, 255, 255), 6)
    return imagen


@pytest.mark.parametrize('corrimiento_m', [0.0, 0.05, -0.05])
def test_con_la_calibracion_el_detector_mide_el_desvio_real(cfg, parametros, corrimiento_m):
    detector = detector_calibrado(parametros, calibrar(cfg, cuadros()))
    medido = detector.detect(carril_recto(corrimiento_m))
    assert medido.detected
    assert medido.lateral_error_m == pytest.approx(corrimiento_m, abs=0.01)
    assert medido.heading_error_rad == pytest.approx(0.0, abs=0.02)


def test_corregir_el_ancho_del_carril_no_cambia_la_medicion(cfg, parametros):
    # El ancho del carril es de la pista: la escala sale solo de la calibración.
    resultado = calibrar(cfg, cuadros())
    imagen = carril_recto(0.05)
    con_037 = detector_calibrado(parametros, resultado, lane_width_m=0.37).detect(imagen)
    con_040 = detector_calibrado(parametros, resultado, lane_width_m=0.40).detect(imagen)
    assert con_040.lateral_error_m == pytest.approx(con_037.lateral_error_m)


def test_cambiar_el_rectangulo_reproyecta_el_trapecio_sin_medir_de_nuevo(cfg, parametros):
    # Lo que reemplaza a arrastrar el trapecio: con la pose ya medida, otro rectángulo da
    # otro trapecio y su escala, y un punto del piso sigue cayendo donde corresponde.
    resultado = calibrar(cfg, cuadros())
    vista = GroundView(0.50, 0.90, 0.22)
    detector = detector_calibrado(parametros, resultado, calib_forward_near_m=vista.near_m,
                                  calib_forward_far_m=vista.far_m,
                                  calib_half_width_m=vista.half_width_m)
    # La pose se guarda redondeada a 4 decimales: el trapecio cambia menos de 0,1 px.
    assert detector.config.warp_src == pytest.approx(
        trapecio(resultado.pose, INTR, vista), abs=2e-4)
    cenital = cv2.perspectiveTransform(
        np.array([[pixel_real(0.70, 0.10)]], dtype=np.float64),
        detector.warp_matrix(ANCHO, ALTO))[0, 0]
    mm_x = vista.width_m / 400
    mm_y = vista.length_m / 400
    assert cenital[0] == pytest.approx(200.0 - 0.10 / mm_x, abs=4.0)
    assert cenital[1] == pytest.approx(400.0 - (0.70 - vista.near_m) / mm_y, abs=4.0)


def test_los_parametros_de_la_calibracion_traen_pose_e_intrinsecos(cfg):
    valores = calibrar(cfg, cuadros()).parametros()
    assert valores['calib_height_m'] == pytest.approx(ALTURA_M, abs=0.005)
    assert valores['calib_pitch_deg'] == pytest.approx(PITCH_DEG, abs=0.5)
    assert valores['calib_roll_deg'] == pytest.approx(0.0, abs=0.3)
    assert intrinsecos_de(valores['camera_intrinsics']) == INTR


def test_el_yaml_trae_una_geometria_valida_de_la_camara_nominal(parametros):
    config = lane_config_desde_parametros(parametros)
    vista = GroundView(parametros['calib_forward_near_m'], parametros['calib_forward_far_m'],
                       parametros['calib_half_width_m'])
    assert config.warp_src == pytest.approx(trapecio(POSE_REAL, INTR, vista), abs=1e-4)
    assert (config.view_width_m, config.view_length_m) == pytest.approx(
        (vista.width_m, vista.length_m))
    # La región de interés no tapa la parte lejana del trapecio.
    assert parametros['roi_top_fraction'] <= min(config.warp_src[1], config.warp_src[3])


# --- validaciones: si alguna falla, no hay resultado ----------------------------------------

def test_sin_profundidad_dice_que_el_piso_no_se_ve(cfg):
    with pytest.raises(CalibrationError, match='el piso no se ve'):
        calibrar(cfg, [np.zeros((ALTO, ANCHO))])


def test_con_un_obstaculo_grande_dice_que_despeje_la_zona(cfg):
    with pytest.raises(CalibrationError, match='hay algo delante de la cámara'):
        calibrar(cfg, cuadros(fraccion_basura=0.7))


def test_una_pared_no_se_toma_por_piso(cfg):
    # Una pared también es un plano, con todos los puntos "inliers" y una "altura" de
    # 0,4 m que pasa el chequeo de altura. La inclinación es la que la delata.
    with pytest.raises(CalibrationError, match='inclinación medida'):
        calibrar(cfg, [pared(0.4)])


def test_una_altura_fuera_de_rango_se_rechaza(cfg):
    with pytest.raises(CalibrationError, match='la altura medida'):
        calibrar(replace(cfg, height_range_m=(0.05, 0.15)), cuadros())


def test_un_rectangulo_cuyo_borde_lejano_no_entra_en_la_imagen_se_rechaza(cfg):
    with pytest.raises(CalibrationError, match='no entra en la imagen.*calib_half_width_m'):
        calibrar(cfg, cuadros(), vista=GroundView(0.45, 0.95, 0.6))


def test_cuadros_de_distinto_tamano_se_rechazan_sin_romper(cfg):
    with pytest.raises(CalibrationError, match='distintos tamaños'):
        calibrar(cfg, [np.zeros((ALTO, ANCHO)), np.zeros((ALTO // 2, ANCHO))])


def test_sin_submuestreo_tambien_calibra(cfg):
    # Con calib_subsample = 1 hay 150 000 puntos: RANSAC por bloques no se queda sin memoria.
    pose = medir_pose(cuadros(n=2), INTR, replace(cfg, subsample=1))
    assert pose.height_m == pytest.approx(ALTURA_M, abs=0.005)


def test_la_profundidad_de_otro_tamano_que_su_camera_info_se_rechaza(cfg):
    with pytest.raises(CalibrationError, match='no coincide'):
        calibrar(cfg, [np.zeros((ALTO // 2, ANCHO // 2))])


@pytest.mark.parametrize('cambios', [
    {'depth_min_m': 2.0},
    {'subsample': 0},
    {'min_inlier_fraction': 0.0},
    {'height_range_m': (0.5, 0.05)},
    {'pitch_range_deg': (10.0,)},
])
def test_una_configuracion_invalida_se_rechaza(cfg, cambios):
    with pytest.raises(ValueError):
        replace(cfg, **cambios)


@pytest.mark.parametrize('cerca, lejos, medio', [(1.0, 0.95, 0.25), (0.45, 0.95, 0.0)])
def test_un_rectangulo_invalido_se_rechaza(cerca, lejos, medio):
    with pytest.raises(ValueError):
        GroundView(cerca, lejos, medio)


@pytest.mark.parametrize('valores', [[615.0, 615.0, 320.0, 240.0], [0.0, 615.0, 320.0, 240.0,
                                                                    640.0, 480.0]])
def test_intrinsecos_invalidos_se_rechazan(valores):
    with pytest.raises(ValueError):
        intrinsecos_de(valores)


def test_el_pitch_de_la_camara_sintetica_es_el_pedido():
    # Control del generador: el ángulo entre el eje óptico y el piso es el pitch.
    normal = normal_del_piso(PITCH_DEG)
    assert math.degrees(math.asin(-normal[2])) == pytest.approx(PITCH_DEG)
