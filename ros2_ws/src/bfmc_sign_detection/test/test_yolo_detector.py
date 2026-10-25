"""Tests del adaptador de YOLO con un módulo `ultralytics` falso.

Ultralytics y PyTorch no están instalados en el CI: se reemplazan por un módulo
que imita lo mínimo que usa el adaptador. Que el modelo real detecte bien las
señales solo se comprueba en la Jetson, con el modelo descargado.
"""
import sys
import types

from bfmc_sign_detection.yolo_detector import crear_detector_yolo, YoloDetector
import numpy as np
import pytest


class Lista:
    """Imita un tensor: solo necesita `tolist`."""

    def __init__(self, datos):
        self._datos = datos

    def tolist(self):
        return self._datos


class ResultadoFalso:

    names = {0: 'stop', 1: 'priority'}

    class boxes:

        xyxy = Lista([[10.5, 20.0, 60.0, 80.0], [100.0, 30.0, 140.0, 90.0]])
        conf = Lista([0.91, 0.55])
        cls = Lista([0.0, 1.0])


class YoloFalso:

    llamadas = []

    def __init__(self, ruta):
        self.ruta = ruta

    def predict(self, imagen, conf, device, verbose):
        YoloFalso.llamadas.append({'conf': conf, 'device': device, 'verbose': verbose})
        return [ResultadoFalso()]


@pytest.fixture
def ultralytics_falso(monkeypatch):
    modulo = types.ModuleType('ultralytics')
    modulo.YOLO = YoloFalso
    monkeypatch.setitem(sys.modules, 'ultralytics', modulo)
    YoloFalso.llamadas = []


@pytest.fixture
def modelo(tmp_path):
    ruta = tmp_path / 'best.pt'
    ruta.write_bytes(b'modelo de prueba')
    return str(ruta)


def test_las_detecciones_del_modelo_se_traducen_al_formato_de_la_logica(
        ultralytics_falso, modelo):
    detecciones = YoloDetector(modelo, 'cpu', 0.25).detect(np.zeros((100, 200, 3), np.uint8))
    assert [d.class_name for d in detecciones] == ['stop', 'priority']
    assert [d.confidence for d in detecciones] == pytest.approx([0.91, 0.55])
    primera = detecciones[0]
    assert (primera.x_min, primera.y_min, primera.x_max, primera.y_max) == (10.5, 20.0, 60.0, 80.0)


def test_el_umbral_y_el_dispositivo_se_pasan_al_modelo(ultralytics_falso, modelo):
    YoloDetector(modelo, 'cuda', 0.3).detect(np.zeros((10, 10, 3), np.uint8))
    assert YoloFalso.llamadas == [{'conf': 0.3, 'device': 'cuda', 'verbose': False}]


def test_un_dispositivo_vacio_deja_que_ultralytics_elija(ultralytics_falso, modelo):
    YoloDetector(modelo, '', 0.3).detect(np.zeros((10, 10, 3), np.uint8))
    assert YoloFalso.llamadas[0]['device'] is None


def test_la_fabrica_por_defecto_crea_el_detector_de_yolo(ultralytics_falso, modelo):
    assert isinstance(crear_detector_yolo(modelo, '', 0.25), YoloDetector)


def test_un_modelo_que_no_existe_dice_como_descargarlo(ultralytics_falso, tmp_path):
    with pytest.raises(FileNotFoundError) as error:
        YoloDetector(str(tmp_path / 'no_esta.pt'), '', 0.25)
    assert 'descargar_modelo' in str(error.value)


def test_la_ruta_del_modelo_expande_el_directorio_personal(
        ultralytics_falso, monkeypatch, tmp_path):
    (tmp_path / 'best.pt').write_bytes(b'x')
    monkeypatch.setenv('HOME', str(tmp_path))
    monkeypatch.setenv('USERPROFILE', str(tmp_path))
    assert YoloDetector('~/best.pt', '', 0.25) is not None


def test_sin_ultralytics_instalado_el_error_dice_como_instalarlo(monkeypatch, modelo):
    monkeypatch.setitem(sys.modules, 'ultralytics', None)
    with pytest.raises(ImportError) as error:
        YoloDetector(modelo, '', 0.25)
    assert 'pip install ultralytics' in str(error.value)
