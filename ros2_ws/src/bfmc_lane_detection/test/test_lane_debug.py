"""Tests de las imágenes de depuración: muestran lo que ve el detector y no lo alteran.

Igual que en test_lane_detection_logic.py, la vista cenital es la identidad: la imagen
de la cámara y la vista desde arriba coinciden, y se sabe dónde está cada línea.
"""
from bfmc_lane_detection.lane_debug import (
    AMARILLO, AZUL, dibujar_camara, dibujar_cenital, MAGENTA, NARANJA, renderizar, ROJO, VERDE,
)
from bfmc_lane_detection.lane_detection_logic import LaneConfig, LaneDetector
import numpy as np
from synthetic_lanes import carril_cenital, lienzo, linea

ANCHO = 200
ALTO = 200


def detector():
    return LaneDetector(LaneConfig(
        white_hsv_min=(0, 0, 200), white_hsv_max=(179, 60, 255), morph_kernel_px=1,
        roi_top_fraction=0.0, warp_src=(0, 0, 1, 0, 1, 1, 0, 1), warp_width=ANCHO,
        warp_height=ALTO, lane_width_m=0.4, lane_width_px=100.0, forward_range_m=0.8,
        n_windows=8, window_margin_px=20, min_pixels=30, reference_row_fraction=0.0,
        full_confidence_pixels=1000, intersection_band_top_fraction=0.5,
        intersection_row_fraction=0.6, intersection_min_rows=5))


def hay_color(imagen, color):
    return bool((imagen == color).all(axis=2).any())


def test_las_imagenes_tienen_el_tamano_y_el_tipo_esperados():
    imagen = carril_cenital(ANCHO, ALTO, 50, 150)
    camara, cenital = renderizar(detector(), imagen)
    assert camara.shape == imagen.shape
    assert cenital.shape == (ALTO, ANCHO, 3)
    assert camara.dtype == np.uint8 and cenital.dtype == np.uint8


def test_dibujar_no_modifica_la_imagen_de_entrada():
    imagen = carril_cenital(ANCHO, ALTO, 50, 150)
    copia = imagen.copy()
    renderizar(detector(), imagen)
    assert np.array_equal(imagen, copia)


def test_lo_que_el_detector_toma_por_blanco_se_pinta_de_rojo_en_la_camara():
    imagen = carril_cenital(ANCHO, ALTO, 50, 150)
    camara, _ = renderizar(detector(), imagen)
    # La columna 47 es parte de la línea (la 50 queda debajo de la curva que se dibuja encima).
    azul, verde, rojo = (int(v) for v in camara[100, 47])
    assert rojo > azul and rojo > verde
    assert tuple(camara[100, 100]) == (0, 0, 0)


def test_se_dibuja_el_trapecio_de_warp_src_en_la_camara():
    camara, _ = renderizar(detector(), lienzo(ANCHO, ALTO))
    assert tuple(camara[0, 100]) == AMARILLO


def test_la_vista_cenital_dibuja_las_ventanas_de_cada_linea():
    _, cenital = renderizar(detector(), carril_cenital(ANCHO, ALTO, 50, 150))
    assert hay_color(cenital, VERDE)
    assert hay_color(cenital, AZUL)


def test_con_una_sola_linea_solo_se_dibujan_las_ventanas_de_ese_lado():
    _, cenital = renderizar(detector(), carril_cenital(ANCHO, ALTO, 50, None))
    assert hay_color(cenital, VERDE)
    assert not hay_color(cenital, AZUL)


def test_se_marcan_la_fila_de_medicion_y_el_centro_del_carril():
    _, cenital = renderizar(detector(), carril_cenital(ANCHO, ALTO, 50, 150))
    assert hay_color(cenital, MAGENTA)
    assert hay_color(cenital, NARANJA)


def test_el_error_lateral_se_dibuja_en_la_camara_cuando_el_carril_esta_corrido():
    camara, _ = renderizar(detector(), carril_cenital(ANCHO, ALTO, 70, 170))
    assert hay_color(camara, MAGENTA)


def test_sin_lineas_se_dibuja_igual_y_no_hay_nada_de_carril():
    camara, cenital = renderizar(detector(), lienzo(ANCHO, ALTO))
    assert not hay_color(camara, ROJO)
    assert not hay_color(cenital, VERDE) and not hay_color(cenital, AZUL)
    assert not hay_color(cenital, MAGENTA)


def test_las_funciones_de_dibujo_se_pueden_usar_con_un_analisis_ya_hecho():
    d = detector()
    imagen = carril_cenital(ANCHO, ALTO, 50, 150)
    analisis = d.analizar(imagen)
    assert dibujar_camara(d, imagen, analisis).shape == imagen.shape
    assert dibujar_cenital(d, analisis).shape == (ALTO, ANCHO, 3)


def test_una_franja_horizontal_cerca_del_auto_se_marca_como_interseccion():
    imagen = carril_cenital(ANCHO, ALTO, 50, 150)
    linea(imagen, (0, 160), (ANCHO - 1, 160), grosor=14)
    _, cenital = renderizar(detector(), imagen)
    assert hay_color(cenital, MAGENTA)
