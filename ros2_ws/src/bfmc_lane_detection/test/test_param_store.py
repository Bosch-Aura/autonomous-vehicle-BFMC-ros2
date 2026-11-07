"""Tests del guardado de parámetros fuera del repo (param_store.py), sin ROS."""
import array
import os

from bfmc_lane_detection.param_store import (
    archivo_persistido, guardar_parametros, leer_parametros,
)
import pytest
import yaml


def test_lo_guardado_se_lee_igual(tmp_path):
    ruta = tmp_path / 'lane.yaml'
    valores = {'lane_width_m': 0.385, 'white_hsv_min': [0, 0, 210],
               'camera_intrinsics': [615.0, 615.0, 320.0, 240.0, 640.0, 480.0]}
    guardar_parametros(str(ruta), 'lane_detection', valores)
    assert leer_parametros(str(ruta), 'lane_detection') == valores


def test_el_archivo_tiene_el_formato_de_un_params_de_ros(tmp_path):
    ruta = tmp_path / 'lane.yaml'
    guardar_parametros(str(ruta), 'lane_detection', {'lane_width_m': 0.4})
    contenido = yaml.safe_load(ruta.read_text(encoding='utf-8'))
    assert contenido == {'lane_detection': {'ros__parameters': {'lane_width_m': 0.4}}}


def test_los_arreglos_de_ros_se_guardan_como_listas(tmp_path):
    # rclpy devuelve los parámetros de tipo arreglo como array.array, que YAML no conoce.
    ruta = tmp_path / 'lane.yaml'
    guardar_parametros(str(ruta), 'n', {'a': array.array('d', [1.5, 2.5]),
                                        'b': array.array('q', [1, 2])})
    assert leer_parametros(str(ruta), 'n') == {'a': [1.5, 2.5], 'b': [1, 2]}


def test_crea_la_carpeta_y_expande_el_home(tmp_path, monkeypatch):
    monkeypatch.setenv('HOME', str(tmp_path))
    ruta = guardar_parametros('~/.config/bfmc/lane.yaml', 'n', {'x': 1.0})
    assert ruta == str(tmp_path / '.config' / 'bfmc' / 'lane.yaml')
    assert os.path.exists(ruta)


def test_guardar_de_nuevo_reemplaza_todo_y_no_deja_temporales(tmp_path):
    ruta = tmp_path / 'lane.yaml'
    guardar_parametros(str(ruta), 'n', {'x': 1.0, 'y': 2.0})
    guardar_parametros(str(ruta), 'n', {'x': 3.0})
    assert leer_parametros(str(ruta), 'n') == {'x': 3.0}
    assert os.listdir(tmp_path) == ['lane.yaml']


def test_si_falla_la_escritura_queda_el_archivo_anterior(tmp_path, monkeypatch):
    ruta = tmp_path / 'lane.yaml'
    guardar_parametros(str(ruta), 'n', {'x': 1.0})

    def explotar(*_args, **_kwargs):
        raise OSError('disco lleno')

    monkeypatch.setattr(yaml, 'safe_dump', explotar)
    with pytest.raises(OSError):
        guardar_parametros(str(ruta), 'n', {'x': 2.0})
    assert leer_parametros(str(ruta), 'n') == {'x': 1.0}
    assert os.listdir(tmp_path) == ['lane.yaml']


def test_sin_archivo_no_hay_nada_guardado(tmp_path):
    assert leer_parametros(str(tmp_path / 'no-existe.yaml'), 'n') == {}


def test_el_launch_solo_carga_el_archivo_si_existe(tmp_path):
    guardado = tmp_path / 'lane.yaml'
    params = tmp_path / 'params.yaml'
    params.write_text(yaml.safe_dump(
        {'lane_detection': {'ros__parameters': {'persist_file': str(guardado)}}}))
    assert archivo_persistido(str(params), 'lane_detection') is None
    guardar_parametros(str(guardado), 'lane_detection', {'lane_width_m': 0.4})
    assert archivo_persistido(str(params), 'lane_detection') == str(guardado)
