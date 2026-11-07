"""Tests de la conversión de mensajes de imagen a arreglos BGR."""
from bfmc_lane_detection.image_utils import imagen_a_bgr, profundidad_a_metros
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


# --- profundidad --------------------------------------------------------------------------

def test_la_profundidad_16uc1_se_pasa_de_milimetros_a_metros():
    mm = np.array([[0, 1000], [450, 1500]], dtype='<u2')
    metros = profundidad_a_metros(2, 2, '16UC1', 4, mm.tobytes())
    assert metros == pytest.approx(np.array([[0.0, 1.0], [0.45, 1.5]]))


def test_la_profundidad_32fc1_ya_viene_en_metros():
    m = np.array([[0.5, 0.75]], dtype='<f4')
    assert profundidad_a_metros(1, 2, '32FC1', 8, m.tobytes()) == pytest.approx(
        np.array([[0.5, 0.75]]))


def test_la_profundidad_respeta_el_relleno_de_cada_fila():
    filas = np.array([[100, 200, 9999], [300, 400, 9999]], dtype='<u2')
    metros = profundidad_a_metros(2, 2, '16UC1', 6, filas.tobytes())
    assert metros == pytest.approx(np.array([[0.1, 0.2], [0.3, 0.4]]))


def test_la_profundidad_big_endian_se_lee_bien():
    mm = np.array([[1234]], dtype='>u2')
    metros = profundidad_a_metros(1, 1, '16UC1', 2, mm.tobytes(), big_endian=True)
    assert metros == pytest.approx(np.array([[1.234]]))


def test_una_profundidad_con_codificacion_no_soportada_se_rechaza():
    with pytest.raises(ValueError, match='no soportada'):
        profundidad_a_metros(1, 1, 'mono8', 1, b'\x00')
