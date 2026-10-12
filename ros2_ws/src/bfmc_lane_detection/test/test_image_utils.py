"""Tests de la conversión de mensajes de imagen a arreglos BGR."""
from bfmc_lane_detection.image_utils import imagen_a_bgr
import numpy as np
import pytest


def pixeles(alto, ancho):
    """Imagen de prueba donde cada canal tiene un valor distinto y reconocible."""
    imagen = np.zeros((alto, ancho, 3), dtype=np.uint8)
    imagen[:, :, 0] = 10
    imagen[:, :, 1] = 20
    imagen[:, :, 2] = 30
    return imagen


def test_una_imagen_bgr8_se_devuelve_sin_cambios():
    original = pixeles(4, 6)
    resultado = imagen_a_bgr(4, 6, 'bgr8', 6 * 3, original.tobytes())
    assert np.array_equal(resultado, original)


def test_una_imagen_rgb8_se_convierte_a_bgr():
    rgb = pixeles(4, 6)
    resultado = imagen_a_bgr(4, 6, 'rgb8', 6 * 3, rgb.tobytes())
    assert np.array_equal(resultado, rgb[:, :, ::-1])
    assert tuple(resultado[0, 0]) == (30, 20, 10)


def test_el_relleno_al_final_de_cada_fila_se_ignora():
    original = pixeles(3, 5)
    relleno = np.full((3, 4), 99, dtype=np.uint8)
    filas = np.concatenate([original.reshape(3, 15), relleno], axis=1)
    resultado = imagen_a_bgr(3, 5, 'bgr8', 19, filas.tobytes())
    assert np.array_equal(resultado, original)


def test_el_resultado_es_utilizable_por_opencv_porque_es_contiguo():
    resultado = imagen_a_bgr(4, 6, 'rgb8', 18, pixeles(4, 6).tobytes())
    assert resultado.flags['C_CONTIGUOUS']


@pytest.mark.parametrize('codificacion', ['mono8', '16UC1', 'bayer_rggb8', ''])
def test_una_codificacion_no_soportada_se_rechaza(codificacion):
    with pytest.raises(ValueError):
        imagen_a_bgr(4, 6, codificacion, 18, bytes(4 * 18))


def test_datos_insuficientes_se_rechazan():
    with pytest.raises(ValueError):
        imagen_a_bgr(4, 6, 'bgr8', 18, bytes(10))


def test_un_paso_menor_al_ancho_de_la_fila_se_rechaza():
    with pytest.raises(ValueError):
        imagen_a_bgr(4, 6, 'bgr8', 10, bytes(4 * 18))
