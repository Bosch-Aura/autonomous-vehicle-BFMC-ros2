"""Tests con capturas reales de la pista (PNG sueltos guardados desde el visor).

Cada archivo de test/capturas/ es un caso. El modo esperado va al principio del nombre,
antes de `__`:

    dos_lineas__recta_01.png      -> MODE_TWO_LINES
    una_linea__reflejo_02.png     -> MODE_ONE_LINE
    perdido__tapada_01.png        -> MODE_LOST (un cuadro suelto sin líneas no tiene pasado)

Se procesa con la configuración real de config/params.yaml. Sin capturas, se saltea.
"""
from pathlib import Path

from bfmc_lane_detection.ground_calibration import lane_config_desde_parametros
from bfmc_lane_detection.lane_detection_logic import LaneDetector
from bfmc_lane_detection.lane_tracker import (
    LaneTracker, MODE_LOST, MODE_ONE_LINE, MODE_TWO_LINES, NOMBRES_DE_MODO,
)
import cv2
import pytest
import yaml

CARPETA = Path(__file__).resolve().parent / 'capturas'
PARAMS = Path(__file__).resolve().parents[1] / 'config' / 'params.yaml'
MODOS = {'dos_lineas': MODE_TWO_LINES, 'una_linea': MODE_ONE_LINE, 'perdido': MODE_LOST}

CAPTURAS = sorted(CARPETA.glob('*.png')) if CARPETA.is_dir() else []
SIN_CAPTURAS = pytest.mark.skip(reason='todavía no hay capturas en test/capturas/')
CASOS = [pytest.param(ruta, id=ruta.stem) for ruta in CAPTURAS] or [
    pytest.param(None, marks=SIN_CAPTURAS)]


@pytest.fixture(scope='module')
def detector():
    with open(PARAMS, encoding='utf-8') as archivo:
        parametros = yaml.safe_load(archivo)['lane_detection']['ros__parameters']
    return LaneDetector(lane_config_desde_parametros(parametros)), parametros['lost_timeout_s']


@pytest.mark.parametrize('ruta', CASOS)
def test_cada_captura_da_el_modo_esperado(detector, ruta):
    prefijo = ruta.name.split('__', 1)[0]
    assert prefijo in MODOS, 'el nombre tiene que empezar con %s y "__"' % ', '.join(MODOS)
    imagen = cv2.imread(str(ruta), cv2.IMREAD_COLOR)
    assert imagen is not None, 'no se pudo leer %s' % ruta.name
    lane_detector, plazo = detector
    resultado = lane_detector.detect(imagen)
    estado = LaneTracker(plazo).update(resultado, 0.0)
    assert estado.mode == MODOS[prefijo], 'modo %s; chequeos que fallaron: %s' % (
        NOMBRES_DE_MODO[estado.mode], resultado.failed_checks or 'ninguno')
