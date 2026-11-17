"""Tests de los detectores intercambiables (ADR-0004).

Todos cumplen el mismo contrato que el detector original: con imágenes sintéticas de
posición conocida miden lo mismo. Además, l_adaptativo tiene que encontrar una línea en
sombra que el umbral fijo en HSV pierde. Misma escala que test_lane_detection_logic.py.
"""
from bfmc_lane_detection import detectores
from bfmc_lane_detection.detectores import HsvVentanas, LAdaptativo
from bfmc_lane_detection.lane_detection_logic import LaneConfig, LaneDetector
import numpy as np
import pytest
from synthetic_lanes import carril_cenital

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


def con_piso(imagen, gris=40):
    """El carril sintético sobre un piso gris (el negro puro no es una pista real)."""
    return np.maximum(imagen, gris).astype(np.uint8)


def con_sombra_a_la_izquierda(imagen, factor=0.6):
    """Oscurece la mitad izquierda, como la sombra de un objeto al costado de la pista."""
    sombreada = imagen.astype(np.float64)
    sombreada[:, :ANCHO // 2] *= factor
    return sombreada.astype(np.uint8)


# --- registro -----------------------------------------------------------------

def test_el_registro_tiene_los_dos_detectores():
    assert detectores.REGISTRO == {'hsv_ventanas': HsvVentanas, 'l_adaptativo': LAdaptativo}


@pytest.mark.parametrize('nombre', sorted(detectores.REGISTRO))
def test_crear_instancia_el_detector_pedido(nombre):
    detector = detectores.crear(configuracion(detector=nombre))
    assert isinstance(detector, detectores.REGISTRO[nombre])
    assert isinstance(detector, LaneDetector)


def test_un_detector_desconocido_se_rechaza():
    with pytest.raises(ValueError, match='detector desconocido'):
        detectores.crear(configuracion(detector='magia'))


def test_hsv_ventanas_es_el_detector_original():
    imagen = con_piso(carril_cenital(ANCHO, ALTO, 50, 150, desvio_arriba=10))
    original = LaneDetector(configuracion()).detect(imagen)
    assert detectores.crear(configuracion()).detect(imagen) == original


# --- el contrato, para cada detector -----------------------------------------------

@pytest.mark.parametrize('nombre', sorted(detectores.REGISTRO))
@pytest.mark.parametrize('x_izquierda, x_derecha, desvio_esperado_px', [
    (50, 150, 0), (70, 170, -20), (30, 130, 20),
])
def test_cada_detector_mide_el_desvio_del_carril(nombre, x_izquierda, x_derecha,
                                                 desvio_esperado_px):
    detector = detectores.crear(configuracion(detector=nombre))
    resultado = detector.detect(con_piso(carril_cenital(ANCHO, ALTO, x_izquierda, x_derecha)))
    assert resultado.detected is True
    assert resultado.lateral_error_m == pytest.approx(desvio_esperado_px * M_POR_PX, abs=0.01)


@pytest.mark.parametrize('nombre', sorted(detectores.REGISTRO))
def test_cada_detector_ve_el_rumbo_de_un_carril_que_dobla(nombre):
    detector = detectores.crear(configuracion(detector=nombre))
    resultado = detector.detect(con_piso(carril_cenital(ANCHO, ALTO, 50, 150, desvio_arriba=30)))
    assert resultado.heading_error_rad < -0.05


@pytest.mark.parametrize('nombre', sorted(detectores.REGISTRO))
def test_cada_detector_sin_lineas_no_detecta_nada(nombre):
    detector = detectores.crear(configuracion(detector=nombre))
    vacia = con_piso(np.zeros((ALTO, ANCHO, 3), dtype=np.uint8))
    assert detector.detect(vacia).detected is False


# --- l_adaptativo con sombra ---------------------------------------------------------

def test_con_sombra_el_umbral_fijo_pierde_la_linea_y_el_adaptativo_no():
    imagen = con_sombra_a_la_izquierda(con_piso(carril_cenital(ANCHO, ALTO, 50, 150)))
    fijo = detectores.crear(configuracion(detector='hsv_ventanas')).detect(imagen)
    adaptativo = detectores.crear(configuracion(detector='l_adaptativo')).detect(imagen)
    assert not fijo.left_valid
    assert adaptativo.left_valid and adaptativo.right_valid
    assert adaptativo.lateral_error_m == pytest.approx(0.0, abs=0.01)


def test_el_piso_l_min_descarta_lo_oscuro_aunque_resalte():
    # Una línea gris oscura resalta sobre el piso negro, pero está por debajo de l_min.
    imagen = carril_cenital(ANCHO, ALTO, 50, 150) // 3
    detector = detectores.crear(configuracion(detector='l_adaptativo', l_min=120))
    assert detector.detect(imagen).detected is False


def test_clahe_no_rompe_la_deteccion():
    detector = detectores.crear(configuracion(detector='l_adaptativo', clahe_clip=2.0))
    resultado = detector.detect(con_piso(carril_cenital(ANCHO, ALTO, 50, 150)))
    assert resultado.left_valid and resultado.right_valid


@pytest.mark.parametrize('cambio', [
    {'l_block_size_px': 24}, {'l_block_size_px': 1}, {'l_offset': -1}, {'l_offset': 256},
    {'l_min': 300}, {'clahe_clip': -1.0},
])
def test_una_configuracion_del_canal_l_invalida_se_rechaza(cambio):
    with pytest.raises(ValueError):
        configuracion(detector='l_adaptativo', **cambio)
