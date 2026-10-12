"""Tests de bfmc_camera: verifican la configuración del driver, sin necesitar la cámara.

El paquete no tiene código propio: solo lanza el driver oficial con parámetros
nuestros. Lo que puede salir mal es un parámetro mal escrito, que el driver
ignoraría en silencio, o que se pierda la profundidad alineada que necesita la
detección de señales. Eso es lo que se verifica acá.
"""
import importlib.util
from pathlib import Path
import re

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import pytest
import yaml

RAIZ = Path(__file__).resolve().parents[1]
PARAMS = RAIZ / 'config' / 'params.yaml'
PERFIL = re.compile(r'^\d+x\d+x\d+$')


def cargar_launch():
    """Carga launch/camera.launch.py desde su ruta y devuelve su descripción."""
    ruta = RAIZ / 'launch' / 'camera.launch.py'
    spec = importlib.util.spec_from_file_location('camera_launch', ruta)
    modulo = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(modulo)
    return modulo.generate_launch_description()


@pytest.fixture(scope='module')
def parametros():
    with open(PARAMS, encoding='utf-8') as archivo:
        return yaml.safe_load(archivo)['camera']['ros__parameters']


def test_el_launch_arranca_el_driver_oficial_de_realsense():
    nodos = [e for e in cargar_launch().entities if isinstance(e, Node)]
    assert len(nodos) == 1
    assert nodos[0].node_package == 'realsense2_camera'
    assert nodos[0].node_executable == 'realsense2_camera_node'


def test_los_parametros_estan_bajo_el_nombre_del_nodo():
    with open(PARAMS, encoding='utf-8') as archivo:
        contenido = yaml.safe_load(archivo)
    assert list(contenido.keys()) == ['camera']
    assert 'ros__parameters' in contenido['camera']


def test_el_yaml_se_instala_con_el_paquete():
    instalado = Path(get_package_share_directory('bfmc_camera')) / 'config' / 'params.yaml'
    assert instalado.is_file()


def test_se_habilitan_el_color_y_la_profundidad(parametros):
    assert parametros['enable_color'] is True
    assert parametros['enable_depth'] is True


def test_la_profundidad_se_alinea_al_color(parametros):
    assert parametros['align_depth.enable'] is True


@pytest.mark.parametrize('clave', ['rgb_camera.color_profile', 'depth_module.depth_profile'])
def test_los_perfiles_tienen_formato_ancho_x_alto_x_fps(parametros, clave):
    assert PERFIL.match(str(parametros[clave]))


def test_todos_los_parametros_existen_en_el_driver(parametros):
    """Un parámetro con el nombre mal escrito se ignora sin avisar: se compara con el driver."""
    carpeta = Path(get_package_share_directory('realsense2_camera')) / 'launch'
    texto = (carpeta / 'rs_launch.py').read_text(encoding='utf-8')
    nombres = set(re.findall(r"'name':\s*'([^']+)'", texto))
    desconocidos = sorted(clave for clave in parametros if clave not in nombres)
    assert desconocidos == []
