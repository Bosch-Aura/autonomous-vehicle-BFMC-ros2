"""Tests de la visión del carril con imágenes sintéticas de posición conocida.

La vista cenital se configura como la identidad (la imagen ya es una vista desde
arriba) para dibujar las líneas donde se quiere y saber qué debe medirse. La
configuración real de la cámara se prueba aparte, en test_lane_detection_yaml.py.

Escala: 100 px = 0,4 m en ambos ejes, es decir 0,004 m por píxel.
"""
import math

from bfmc_lane_detection.lane_detection_logic import (
    cuadrilatero_valido, LaneConfig, LaneDetector, Linea, PRIOR_SEARCH, SLIDING_WINDOWS,
)
import cv2
import numpy as np
import pytest
from synthetic_lanes import carril_cenital, lienzo, linea

ANCHO = 200
ALTO = 200
M_POR_PX = 0.004


def configuracion(**cambios):
    base = {
        'white_hsv_min': (0, 0, 200), 'white_hsv_max': (179, 60, 255), 'morph_kernel_px': 1,
        'roi_top_fraction': 0.0, 'warp_src': (0, 0, 1, 0, 1, 1, 0, 1),
        'warp_width': ANCHO, 'warp_height': ALTO, 'lane_width_m': 0.4, 'view_width_m': 0.8,
        'view_length_m': 0.8, 'n_windows': 8, 'window_margin_px': 20, 'min_pixels': 30,
        'min_windows': 3, 'max_residual_m': 0.02, 'min_curve_radius_m': 0.4,
        'lane_width_tolerance_m': 0.05, 'width_check_fractions': (0.1, 0.5, 0.9),
        'search_margin_m': 0.05,
        'reference_row_fraction': 0.0, 'full_confidence_pixels': 1000,
        'intersection_band_top_fraction': 0.5, 'intersection_row_fraction': 0.6,
        'intersection_min_rows': 5,
    }
    return LaneConfig(**{**base, **cambios})


def detectar(imagen, **cambios):
    return LaneDetector(configuracion(**cambios)).detect(imagen)


# --- desvío lateral ---------------------------------------------------------

def test_un_auto_centrado_en_el_carril_no_tiene_desvio():
    resultado = detectar(carril_cenital(ANCHO, ALTO, 50, 150))
    assert resultado.detected is True
    assert resultado.lateral_error_m == pytest.approx(0.0, abs=0.01)
    assert resultado.heading_error_rad == pytest.approx(0.0, abs=0.02)


def test_si_el_centro_del_carril_queda_a_la_derecha_el_desvio_es_negativo():
    resultado = detectar(carril_cenital(ANCHO, ALTO, 70, 170))
    assert resultado.lateral_error_m == pytest.approx(-20 * M_POR_PX, abs=0.01)


def test_si_el_centro_del_carril_queda_a_la_izquierda_el_desvio_es_positivo():
    resultado = detectar(carril_cenital(ANCHO, ALTO, 30, 130))
    assert resultado.lateral_error_m == pytest.approx(20 * M_POR_PX, abs=0.01)


def test_el_ancho_del_carril_no_cambia_la_escala():
    # La escala es de la cámara (view_width_m). Corregir el ancho medido del carril no
    # tiene que mover el desvío medido con las dos líneas a la vista.
    imagen = carril_cenital(ANCHO, ALTO, 70, 170)
    assert detectar(imagen, lane_width_m=0.38).lateral_error_m == pytest.approx(
        detectar(imagen, lane_width_m=0.42).lateral_error_m)


def test_el_ancho_del_carril_en_pixeles_se_calcula_con_la_escala():
    assert LaneDetector(configuracion(lane_width_m=0.4)).lane_width_px == pytest.approx(100.0)


# --- rumbo ------------------------------------------------------------------

def test_un_carril_que_sigue_hacia_la_derecha_tiene_rumbo_negativo():
    resultado = detectar(carril_cenital(ANCHO, ALTO, 50, 150, desvio_arriba=30))
    assert resultado.heading_error_rad == pytest.approx(math.atan(-30 / 199), abs=0.03)


def test_un_carril_que_sigue_hacia_la_izquierda_tiene_rumbo_positivo():
    resultado = detectar(carril_cenital(ANCHO, ALTO, 50, 150, desvio_arriba=-30))
    assert resultado.heading_error_rad == pytest.approx(math.atan(30 / 199), abs=0.03)


def test_el_desvio_se_mide_en_la_fila_de_referencia_elegida():
    imagen = carril_cenital(ANCHO, ALTO, 50, 150, desvio_arriba=40)
    pegado = detectar(imagen, reference_row_fraction=0.0)
    adelante = detectar(imagen, reference_row_fraction=0.5)
    # Más adelante el carril se corrió hacia la derecha: el desvío es más negativo.
    assert adelante.lateral_error_m < pegado.lateral_error_m - 0.02


# --- una sola línea ---------------------------------------------------------

def test_con_solo_la_linea_izquierda_se_estima_el_centro_con_el_ancho_del_carril():
    resultado = detectar(carril_cenital(ANCHO, ALTO, 50, None))
    assert resultado.detected is True
    assert resultado.lateral_error_m == pytest.approx(0.0, abs=0.01)


def test_con_solo_la_linea_derecha_se_estima_el_centro_con_el_ancho_del_carril():
    resultado = detectar(carril_cenital(ANCHO, ALTO, None, 150))
    assert resultado.detected is True
    assert resultado.lateral_error_m == pytest.approx(0.0, abs=0.01)


def test_dos_lineas_demasiado_juntas_se_toman_como_una_sola():
    resultado = detectar(carril_cenital(ANCHO, ALTO, 90, 110, grosor=8))
    assert resultado.detected is True
    assert resultado.confidence <= 0.5


# --- confianza --------------------------------------------------------------

def test_con_las_dos_lineas_la_confianza_es_mayor_que_con_una():
    ambas = detectar(carril_cenital(ANCHO, ALTO, 50, 150))
    una = detectar(carril_cenital(ANCHO, ALTO, 50, None))
    assert ambas.confidence > una.confidence


def test_la_confianza_crece_con_la_cantidad_de_pixeles_de_linea():
    poca = detectar(carril_cenital(ANCHO, ALTO, 50, 150, grosor=2), full_confidence_pixels=3000)
    mucha = detectar(carril_cenital(ANCHO, ALTO, 50, 150, grosor=12), full_confidence_pixels=3000)
    assert 0.0 < poca.confidence < mucha.confidence <= 1.0


def test_la_confianza_nunca_supera_uno():
    assert detectar(carril_cenital(ANCHO, ALTO, 50, 150, grosor=14),
                    full_confidence_pixels=10).confidence == 1.0


# --- sin carril -------------------------------------------------------------

def test_una_imagen_sin_lineas_no_detecta_el_carril():
    resultado = detectar(lienzo(ANCHO, ALTO))
    assert resultado.detected is False
    assert resultado.confidence == 0.0
    assert resultado.lateral_error_m == 0.0


def test_una_mancha_chica_no_se_toma_por_una_linea():
    imagen = lienzo(ANCHO, ALTO)
    linea(imagen, (50, 100), (53, 103), grosor=3)
    assert detectar(imagen).detected is False


def test_las_lineas_que_no_son_blancas_se_ignoran():
    imagen = lienzo(ANCHO, ALTO)
    for x in (50, 150):
        linea(imagen, (x, ALTO - 1), (x, 0), color=(0, 255, 255))   # amarillas
    assert detectar(imagen).detected is False


def test_las_lineas_poco_brillantes_se_ignoran():
    imagen = lienzo(ANCHO, ALTO)
    for x in (50, 150):
        linea(imagen, (x, ALTO - 1), (x, 0), color=(120, 120, 120))
    assert detectar(imagen).detected is False


def test_la_region_de_interes_descarta_la_parte_alta_de_la_imagen():
    """La región de interés descarta la parte alta de la imagen.

    Con las líneas a lo largo de toda la imagen, descartar el 90 % de arriba deja muy
    pocos píxeles y la línea ya no alcanza el mínimo; sin descartar nada sí se detecta.
    """
    imagen = carril_cenital(ANCHO, ALTO, 50, 150)
    assert detectar(imagen, roi_top_fraction=0.0, min_pixels=90).detected is True
    assert detectar(imagen, roi_top_fraction=0.9, min_pixels=90).detected is False


# --- intersección -----------------------------------------------------------

def test_una_franja_horizontal_cerca_del_auto_se_detecta_como_interseccion():
    imagen = carril_cenital(ANCHO, ALTO, 50, 150)
    linea(imagen, (0, 150), (ANCHO - 1, 150), grosor=12)
    resultado = detectar(imagen)
    assert resultado.intersection_detected is True
    assert resultado.detected is True


def test_un_carril_sin_franja_horizontal_no_es_una_interseccion():
    assert detectar(carril_cenital(ANCHO, ALTO, 50, 150)).intersection_detected is False


def test_una_franja_lejana_no_se_detecta_como_interseccion_todavia():
    imagen = carril_cenital(ANCHO, ALTO, 50, 150)
    linea(imagen, (0, 30), (ANCHO - 1, 30), grosor=12)
    assert detectar(imagen).intersection_detected is False


def test_una_franja_angosta_no_alcanza():
    imagen = carril_cenital(ANCHO, ALTO, 50, 150)
    linea(imagen, (0, 150), (ANCHO - 1, 150), grosor=2)
    assert detectar(imagen).intersection_detected is False


# --- entrada y configuración ------------------------------------------------

def test_una_imagen_que_no_es_bgr_se_rechaza():
    with pytest.raises(ValueError):
        detectar(np.zeros((ALTO, ANCHO), dtype=np.uint8))


def test_el_mismo_cuadro_da_siempre_el_mismo_resultado():
    imagen = carril_cenital(ANCHO, ALTO, 60, 160, desvio_arriba=10)
    detector = LaneDetector(configuracion())
    assert detector.detect(imagen) == detector.detect(imagen)


def test_la_matriz_de_perspectiva_se_calcula_una_sola_vez_por_tamano():
    detector = LaneDetector(configuracion())
    assert detector.warp_matrix(640, 480) is detector.warp_matrix(640, 480)


@pytest.mark.parametrize('cambio', [
    {'white_hsv_min': (0, 0)},
    {'white_hsv_min': (0, 0, 250), 'white_hsv_max': (179, 60, 200)},
    {'morph_kernel_px': 0},
    {'roi_top_fraction': 1.0},
    {'warp_src': (0, 0, 1, 0)},
    {'warp_src': (0, 0, 1, 0, 1, 1, 0, 2)},
    {'warp_width': 1},
    {'lane_width_m': 0.0},
    {'view_length_m': -1.0},
    {'view_width_m': 0.4},          # el carril no entra en la vista
    {'n_windows': 0},
    {'min_pixels': 0},
    {'reference_row_fraction': 1.0},
    {'full_confidence_pixels': 0},
    {'intersection_row_fraction': 0.0},
    {'intersection_min_rows': 0},
])
def test_una_configuracion_invalida_se_rechaza(cambio):
    with pytest.raises(ValueError):
        configuracion(**cambio)


# --- trapecio de la vista cenital ---------------------------------------------

@pytest.mark.parametrize('warp_src', [
    (0.0, 0.5, 1.0, 0.5, 1.0, 0.5, 0.0, 0.5),   # los cuatro puntos en una recta
    (0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 1.0),   # lados cruzados
    (0.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0),   # orden invertido: abajo antes que arriba
    (0.5, 0.0, 0.5, 0.0, 1.0, 1.0, 0.0, 1.0),   # dos puntos iguales
])
def test_un_trapecio_degenerado_o_cruzado_se_rechaza(warp_src):
    assert cuadrilatero_valido(warp_src) is False
    with pytest.raises(ValueError):
        configuracion(warp_src=warp_src)


def test_un_trapecio_normal_es_valido():
    assert cuadrilatero_valido((0, 0, 1, 0, 1, 1, 0, 1)) is True
    assert cuadrilatero_valido((0.30, 0.55, 0.70, 0.55, 1.00, 0.95, 0.00, 0.95)) is True


# --- lo que el detector vio -----------------------------------------------------

def test_analizar_da_el_mismo_resultado_que_detect():
    detector = LaneDetector(configuracion())
    imagenes = [
        carril_cenital(ANCHO, ALTO, 50, 150),
        carril_cenital(ANCHO, ALTO, 70, 170, desvio_arriba=20),
        carril_cenital(ANCHO, ALTO, 50, None),
        lienzo(ANCHO, ALTO),
    ]
    for imagen in imagenes:
        assert detector.analizar(imagen).resultado == detector.detect(imagen)


def test_analizar_registra_las_ventanas_de_cada_linea():
    analisis = LaneDetector(configuracion()).analizar(carril_cenital(ANCHO, ALTO, 50, 150))
    for lado in ('izq', 'der'):
        assert len(analisis.trazas[lado]['ventanas']) == 8
    assert analisis.trazas['izq']['base'] == pytest.approx(50, abs=5)
    assert analisis.trazas['der']['base'] == pytest.approx(150, abs=5)


def test_analizar_sin_la_linea_derecha_no_registra_ventanas_de_ese_lado():
    analisis = LaneDetector(configuracion()).analizar(carril_cenital(ANCHO, ALTO, 50, None))
    assert analisis.derecha is None
    assert analisis.trazas['der'] == {'base': None, 'ventanas': []}


def test_analizar_expone_la_mascara_de_blancos_y_la_vista_cenital():
    imagen = carril_cenital(ANCHO, ALTO, 50, 150)
    analisis = LaneDetector(configuracion()).analizar(imagen)
    assert analisis.mascara.shape == imagen.shape[:2]
    assert analisis.cenital.shape == (ALTO, ANCHO)
    assert (analisis.mascara > 0).any()


def test_analizar_indica_con_que_lineas_se_midio():
    detector = LaneDetector(configuracion())
    assert detector.analizar(carril_cenital(ANCHO, ALTO, 50, 150)).centro['modo'] == 'ambas'
    assert detector.analizar(carril_cenital(ANCHO, ALTO, 50, None)).centro['modo'] == 'izquierda'
    assert detector.analizar(carril_cenital(ANCHO, ALTO, None, 150)).centro['modo'] == 'derecha'
    assert detector.analizar(lienzo(ANCHO, ALTO)).centro is None


# --- chequeos de cada medición (A2-152) -----------------------------------------------

def zigzag(imagen, x, amplitud, grosor=2):
    """Una línea vertical en `x` que se va corriendo ± `amplitud` px: ajusta mal."""
    puntos = [(x + (amplitud if (y // 10) % 2 else -amplitud), y) for y in range(ALTO, -1, -10)]
    cv2.polylines(imagen, [np.array(puntos, dtype=np.int32).reshape(-1, 1, 2)], False,
                  (255, 255, 255), grosor)
    return imagen


def test_un_carril_bueno_tiene_las_dos_lineas_validas():
    resultado = detectar(carril_cenital(ANCHO, ALTO, 50, 150))
    assert (resultado.left_valid, resultado.right_valid) == (True, True)
    assert resultado.failed_checks == ''


def test_un_reflejo_que_rompe_el_ancho_se_descarta_y_queda_la_linea_buena():
    # Línea buena a la izquierda; a la derecha, un "reflejo" desparejo a 0,52 m.
    imagen = zigzag(carril_cenital(ANCHO, ALTO, 50, None, grosor=2), 180, 6, grosor=6)
    resultado = detectar(imagen)
    assert resultado.detected is True
    assert (resultado.left_valid, resultado.right_valid) == (True, False)
    assert resultado.failed_checks
    # El centro sale de la línea buena más medio carril: el auto está centrado.
    assert resultado.lateral_error_m == pytest.approx(0.0, abs=0.01)


def test_si_el_ancho_no_cierra_se_queda_la_linea_que_mejor_ajusta():
    # Las dos pasan sus chequeos, pero están a 0,52 m: se descarta la de mayor residuo
    # aunque tenga más píxeles.
    imagen = zigzag(carril_cenital(ANCHO, ALTO, 40, None, grosor=2), 170, 3, grosor=4)
    resultado = detectar(imagen)
    assert 'ancho' in resultado.failed_checks
    assert (resultado.left_valid, resultado.right_valid) == (True, False)


def test_una_linea_corta_con_pocas_ventanas_se_descarta():
    imagen = linea(lienzo(ANCHO, ALTO), (50, ALTO - 1), (50, ALTO - 45), grosor=8)
    resultado = detectar(imagen)
    assert resultado.detected is False
    assert 'pocas ventanas' in resultado.failed_checks


def test_una_linea_mas_curva_que_la_pista_se_descarta():
    imagen = lienzo(ANCHO, ALTO)
    puntos = [(int(50 + 0.002 * (ALTO - y) ** 2), y) for y in range(ALTO, -1, -5)]
    cv2.polylines(imagen, [np.array(puntos, dtype=np.int32).reshape(-1, 1, 2)], False,
                  (255, 255, 255), 6)
    # Curvatura en la fila de medición: 2 * 0,002 px⁻¹ = 1 m⁻¹ (radio 1 m).
    assert detectar(imagen, min_curve_radius_m=0.4).detected is True
    curva = detectar(imagen, min_curve_radius_m=5.0)
    assert curva.detected is False
    assert 'curva' in curva.failed_checks


@pytest.mark.parametrize('separacion_m, angulo_deg, esperado', [
    (0.4 / math.cos(math.radians(30)), 30.0, True),   # carril de 0,40 m inclinado 30°
    (0.40, 30.0, False),                              # 0,40 horizontal = 0,35 perpendicular
    (0.40, 0.0, True),
])
def test_el_ancho_se_mide_perpendicular_a_las_lineas(separacion_m, angulo_deg, esperado):
    # En una curva las líneas cruzan la fila inclinadas: en horizontal el carril "mide"
    # más. Sin corregir por el ángulo, una curva cerrada perdería siempre una línea.
    detector = LaneDetector(configuracion())
    pendiente = math.tan(math.radians(angulo_deg))
    separacion_px = separacion_m / M_POR_PX
    izquierda = Linea(np.array([0.0, pendiente, 50.0]), 1000, 8, 0.5)
    derecha = Linea(np.array([0.0, pendiente, 50.0 + separacion_px]), 1000, 8, 0.5)
    assert detector._ancho_ok(izquierda, derecha) is esperado


# --- búsqueda alrededor del polinomio previo (A2-153) -------------------------------

def test_sin_previo_se_buscan_las_ventanas_deslizantes():
    resultado = detectar(carril_cenital(ANCHO, ALTO, 50, 150))
    assert (resultado.left_method, resultado.right_method) == (
        SLIDING_WINDOWS, SLIDING_WINDOWS)
    assert resultado.left_fit is not None and resultado.right_fit is not None


def test_si_la_linea_se_corre_2_cm_la_busqueda_cerca_del_previo_la_encuentra():
    detector = LaneDetector(configuracion())
    primero = detector.detect(carril_cenital(ANCHO, ALTO, 50, 150))
    corrimiento_px = int(round(0.02 / M_POR_PX))       # 2 cm = 5 px
    segundo = detector.detect(
        carril_cenital(ANCHO, ALTO, 50 + corrimiento_px, 150 + corrimiento_px),
        previas=(primero.left_fit, primero.right_fit))
    assert (segundo.left_method, segundo.right_method) == (PRIOR_SEARCH, PRIOR_SEARCH)
    assert segundo.lateral_error_m == pytest.approx(primero.lateral_error_m - 0.02, abs=0.005)


def test_un_reflejo_lejos_del_previo_no_se_toma():
    detector = LaneDetector(configuracion())
    primero = detector.detect(carril_cenital(ANCHO, ALTO, 50, 150))
    imagen = carril_cenital(ANCHO, ALTO, 50, 150)
    cv2.rectangle(imagen, (175, 0), (195, ALTO - 1), (255, 255, 255), -1)   # reflejo
    segundo = detector.detect(imagen, previas=(primero.left_fit, primero.right_fit))
    assert segundo.right_method == PRIOR_SEARCH
    assert segundo.right_valid is True
    # La derecha sigue en x = 150 en la fila de abajo, no en el reflejo.
    assert np.polyval(segundo.right_fit, ALTO - 1) == pytest.approx(150, abs=3)
    assert segundo.lateral_error_m == pytest.approx(primero.lateral_error_m, abs=0.005)


def test_si_no_hay_nada_cerca_del_previo_se_vuelve_a_las_ventanas_en_el_mismo_cuadro():
    detector = LaneDetector(configuracion())
    primero = detector.detect(carril_cenital(ANCHO, ALTO, 50, 150))
    # La izquierda desapareció de donde estaba: no hay nada cerca del previo.
    segundo = detector.detect(carril_cenital(ANCHO, ALTO, None, 150),
                              previas=(primero.left_fit, primero.right_fit))
    assert segundo.left_method == SLIDING_WINDOWS
    assert segundo.right_method == PRIOR_SEARCH      # las reglas son por línea
    assert segundo.left_valid is False


def test_el_ancho_descartado_se_informa_para_olvidar_ese_previo():
    imagen = zigzag(carril_cenital(ANCHO, ALTO, 40, None, grosor=2), 170, 3, grosor=4)
    assert detectar(imagen).width_discarded == 'right'


# --- curvatura (A2-154) --------------------------------------------------------------

def carril_curvo(k):
    """Dos líneas x = x0 - k (ALTO - y)²: con k > 0 el carril dobla a la izquierda."""
    imagen = lienzo(ANCHO, ALTO)
    for x0 in (50, 150):
        puntos = [(int(round(x0 - k * (ALTO - y) ** 2)), y) for y in range(ALTO, -1, -5)]
        cv2.polylines(imagen, [np.array(puntos, dtype=np.int32).reshape(-1, 1, 2)], False,
                      (255, 255, 255), 6)
    return imagen


@pytest.mark.parametrize('k, esperada', [(0.0005, 0.25), (-0.0005, -0.25), (0.0, 0.0)])
def test_la_curvatura_es_positiva_si_el_carril_dobla_a_la_izquierda(k, esperada):
    # En la fila de medición (abajo, pendiente 0): curvatura = 2 k [px⁻¹] / 0,004 m/px.
    resultado = detectar(carril_curvo(k))
    assert resultado.detected is True
    assert resultado.curvature == pytest.approx(esperada, abs=0.05)
