"""Tests de la imagen de depuración de señales."""
import math

from bfmc_sign_detection.sign_debug import COLOR_CAJA_BGR, dibujar_senales, texto_de_senal
from bfmc_sign_detection.sign_detection_logic import SignResult
import numpy as np


def senal(distancia=1.2):
    return SignResult('stop', 0.87, distancia, 40, 50, 100, 90)


def imagen_negra():
    return np.zeros((120, 160, 3), dtype=np.uint8)


def test_el_texto_lleva_etiqueta_confianza_y_distancia():
    assert texto_de_senal(senal()) == 'stop 0.87 | 1.2 m'


def test_sin_distancia_el_texto_no_la_menciona():
    assert texto_de_senal(senal(math.nan)) == 'stop 0.87'


def test_sin_senales_la_imagen_queda_igual():
    imagen = imagen_negra()
    assert np.array_equal(dibujar_senales(imagen, []), imagen)


def test_dibuja_la_caja_sin_tocar_la_imagen_original():
    imagen = imagen_negra()
    dibujada = dibujar_senales(imagen, [senal()])
    assert not imagen.any()
    assert dibujada.shape == imagen.shape
    # El borde izquierdo de la caja, a media altura, tiene el color de la caja.
    assert tuple(dibujada[70, 40]) == COLOR_CAJA_BGR
    # Y el interior de la caja no se pinta.
    assert not dibujada[70, 70].any()


def test_una_caja_pegada_al_borde_de_arriba_no_rompe():
    pegada = SignResult('stop', 0.9, math.nan, 10, 0, 50, 30)
    assert dibujar_senales(imagen_negra(), [pegada]).any()
