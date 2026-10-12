"""Tests con la configuración REAL de config/params.yaml.

Se dibuja un carril en vista cenital de posición conocida, se lo proyecta a la
imagen de la cámara con la inversa de la misma transformación del YAML y se
comprueba que el pipeline recupere el desvío y el rumbo. Si alguien cambia el
YAML y rompe la geometría, estos tests fallan.

No prueban que la configuración sea la correcta para la cámara real: eso solo
se sabe con imágenes de la pista.
"""
from pathlib import Path

import pytest
from synthetic_lanes import a_vista_de_camara, carril_cenital, lienzo, linea
import yaml

from bfmc_lane_detection.lane_detection_logic import LaneConfig, LaneDetector

PARAMS = Path(__file__).resolve().parents[1] / 'config' / 'params.yaml'
CAMARA_ANCHO = 640
CAMARA_ALTO = 480
TOLERANCIA_M = 0.03


@pytest.fixture(scope='module')
def parametros():
    with open(PARAMS, encoding='utf-8') as archivo:
        return yaml.safe_load(archivo)['lane_detection']['ros__parameters']


@pytest.fixture(scope='module')
def detector(parametros):
    claves = [k for k in LaneConfig.__dataclass_fields__]
    return LaneDetector(LaneConfig(**{k: parametros[k] for k in claves}))


@pytest.fixture(scope='module')
def geometria(parametros):
    return parametros['warp_width'], parametros['warp_height'], parametros['lane_width_px']


def lineas(centro, carril):
    """Posición (x) de las líneas izquierda y derecha de un carril con ese centro."""
    return int(centro - carril / 2), int(centro + carril / 2)


def vista_de_camara(detector, geometria, x_izquierda, x_derecha, desvio_arriba=0):
    ancho, alto, _ = geometria
    cenital = carril_cenital(ancho, alto, x_izquierda, x_derecha, desvio_arriba, grosor=10)
    matriz = detector.warp_matrix(CAMARA_ANCHO, CAMARA_ALTO)
    return a_vista_de_camara(cenital, matriz, CAMARA_ANCHO, CAMARA_ALTO)


def test_el_yaml_trae_una_configuracion_valida(detector):
    assert detector is not None


def test_un_carril_centrado_se_mide_sin_desvio(detector, geometria):
    ancho, _, carril = geometria
    centro = ancho / 2
    frame = vista_de_camara(detector, geometria, *lineas(centro, carril))
    resultado = detector.detect(frame)
    assert resultado.detected is True
    assert resultado.lateral_error_m == pytest.approx(0.0, abs=TOLERANCIA_M)


def test_un_carril_corrido_a_la_derecha_da_desvio_negativo(detector, geometria):
    ancho, _, carril = geometria
    centro = ancho / 2 + 40
    frame = vista_de_camara(detector, geometria, *lineas(centro, carril))
    resultado = detector.detect(frame)
    esperado = -40 * (0.37 / carril)
    assert resultado.detected is True
    assert resultado.lateral_error_m == pytest.approx(esperado, abs=TOLERANCIA_M)


def test_un_carril_corrido_a_la_izquierda_da_desvio_positivo(detector, geometria):
    ancho, _, carril = geometria
    centro = ancho / 2 - 40
    frame = vista_de_camara(detector, geometria, *lineas(centro, carril))
    resultado = detector.detect(frame)
    assert resultado.lateral_error_m > 0.02


def test_un_carril_que_sigue_hacia_la_derecha_da_rumbo_negativo(detector, geometria):
    ancho, _, carril = geometria
    centro = ancho / 2
    frame = vista_de_camara(detector, geometria, *lineas(centro, carril), desvio_arriba=60)
    assert detector.detect(frame).heading_error_rad < -0.05


def test_un_carril_que_sigue_hacia_la_izquierda_da_rumbo_positivo(detector, geometria):
    ancho, _, carril = geometria
    centro = ancho / 2
    frame = vista_de_camara(detector, geometria, *lineas(centro, carril), desvio_arriba=-60)
    assert detector.detect(frame).heading_error_rad > 0.05


def test_una_imagen_negra_no_detecta_el_carril(detector):
    assert detector.detect(lienzo(CAMARA_ANCHO, CAMARA_ALTO)).detected is False


def test_una_franja_horizontal_en_el_piso_se_detecta_como_interseccion(detector, geometria):
    ancho, alto, carril = geometria
    centro = ancho / 2
    cenital = carril_cenital(ancho, alto, *lineas(centro, carril), grosor=10)
    linea(cenital, (0, int(alto * 0.8)), (ancho - 1, int(alto * 0.8)), grosor=14)
    frame = a_vista_de_camara(
        cenital, detector.warp_matrix(CAMARA_ANCHO, CAMARA_ALTO), CAMARA_ANCHO, CAMARA_ALTO)
    resultado = detector.detect(frame)
    assert resultado.intersection_detected is True
    assert resultado.detected is True
