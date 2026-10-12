"""Tests de la conversión de mensajes de imagen de color y de profundidad."""
import math

import numpy as np
import pytest

from bfmc_sign_detection.image_utils import imagen_a_bgr, profundidad_a_metros


def color(alto, ancho):
    imagen = np.zeros((alto, ancho, 3), dtype=np.uint8)
    imagen[:, :, 0], imagen[:, :, 1], imagen[:, :, 2] = 10, 20, 30
    return imagen


# --- color --------------------------------------------------------------------------------

def test_una_imagen_bgr8_se_devuelve_sin_cambios():
    original = color(4, 6)
    assert np.array_equal(imagen_a_bgr(4, 6, 'bgr8', 18, original.tobytes()), original)


def test_una_imagen_rgb8_se_convierte_a_bgr():
    resultado = imagen_a_bgr(4, 6, 'rgb8', 18, color(4, 6).tobytes())
    assert tuple(resultado[0, 0]) == (30, 20, 10)
    assert resultado.flags['C_CONTIGUOUS']


@pytest.mark.parametrize('codificacion', ['mono8', '16UC1', ''])
def test_una_codificacion_de_color_no_soportada_se_rechaza(codificacion):
    with pytest.raises(ValueError):
        imagen_a_bgr(4, 6, codificacion, 18, bytes(72))


def test_color_con_datos_insuficientes_se_rechaza():
    with pytest.raises(ValueError):
        imagen_a_bgr(4, 6, 'bgr8', 18, bytes(10))


# --- profundidad ----------------------------------------------------------------------------

def profundidad_16(valores, bigendian=False, relleno=0):
    alto, ancho = valores.shape
    tipo = '>u2' if bigendian else '<u2'
    filas = valores.astype(tipo).view(np.uint8).reshape(alto, ancho * 2)
    if relleno:
        filas = np.concatenate([filas, np.full((alto, relleno), 77, dtype=np.uint8)], axis=1)
    return filas.tobytes(), filas.shape[1]


def test_la_profundidad_16uc1_en_milimetros_se_convierte_a_metros():
    valores = np.array([[1000, 2500], [500, 4000]], dtype=np.uint16)
    datos, paso = profundidad_16(valores)
    metros = profundidad_a_metros(2, 2, '16UC1', paso, False, datos, 0.001)
    assert metros.dtype == np.float32
    np.testing.assert_allclose(metros, [[1.0, 2.5], [0.5, 4.0]])


def test_un_cero_en_la_profundidad_16uc1_significa_sin_medicion():
    valores = np.array([[0, 1500]], dtype=np.uint16)
    datos, paso = profundidad_16(valores)
    metros = profundidad_a_metros(1, 2, '16UC1', paso, False, datos, 0.001)
    assert math.isnan(metros[0, 0])
    assert metros[0, 1] == pytest.approx(1.5)


def test_la_profundidad_respeta_el_orden_de_bytes_del_mensaje():
    valores = np.array([[1000, 2000]], dtype=np.uint16)
    datos, paso = profundidad_16(valores, bigendian=True)
    metros = profundidad_a_metros(1, 2, '16UC1', paso, True, datos, 0.001)
    assert metros[0].tolist() == pytest.approx([1.0, 2.0])


def test_la_profundidad_32fc1_ya_esta_en_metros():
    valores = np.array([[1.25, 3.5]], dtype='<f4')
    metros = profundidad_a_metros(1, 2, '32FC1', 8, False, valores.tobytes(), 0.001)
    assert metros[0].tolist() == pytest.approx([1.25, 3.5])


def test_el_relleno_al_final_de_cada_fila_de_profundidad_se_ignora():
    valores = np.array([[1000, 2000], [3000, 4000]], dtype=np.uint16)
    datos, paso = profundidad_16(valores, relleno=6)
    metros = profundidad_a_metros(2, 2, '16UC1', paso, False, datos, 0.001)
    np.testing.assert_allclose(metros, [[1.0, 2.0], [3.0, 4.0]])


@pytest.mark.parametrize('codificacion', ['bgr8', 'mono16', '8UC1', ''])
def test_una_codificacion_de_profundidad_no_soportada_se_rechaza(codificacion):
    with pytest.raises(ValueError):
        profundidad_a_metros(1, 2, codificacion, 4, False, bytes(4), 0.001)


def test_profundidad_con_datos_insuficientes_se_rechaza():
    with pytest.raises(ValueError):
        profundidad_a_metros(2, 2, '16UC1', 4, False, bytes(3), 0.001)
