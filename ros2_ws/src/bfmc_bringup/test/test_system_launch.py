"""Tests de la descripción del launch: qué nodos arranca y qué opciones tiene."""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
import pytest
import yaml

LAUNCH = Path(__file__).resolve().parents[1] / 'launch' / 'system.launch.py'

# paquete -> ejecutable de cada nodo del sistema
NODOS = {
    'bfmc_lane_detection': 'lane_detection',
    'bfmc_sign_detection': 'sign_detection',
    'bfmc_control': 'control',
    'bfmc_hardware_bridge': 'hardware_bridge',
    'bfmc_maneuvers': 'maneuvers',
    'bfmc_decision': 'decision',
}
OPCIONALES = {'bfmc_sign_detection'}


@pytest.fixture(scope='module')
def descripcion():
    spec = importlib.util.spec_from_file_location('system_launch', LAUNCH)
    modulo = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(modulo)
    return modulo.generate_launch_description()


def nodos(descripcion):
    return [e for e in descripcion.entities if isinstance(e, Node)]


def test_el_launch_arranca_un_nodo_por_cada_paquete_de_la_arquitectura(descripcion):
    arrancados = {n.node_package: n.node_executable for n in nodos(descripcion)}
    assert arrancados == NODOS


def test_no_hay_nodos_repetidos(descripcion):
    ejecutables = [n.node_executable for n in nodos(descripcion)]
    assert len(ejecutables) == len(set(ejecutables))


def test_solo_la_deteccion_de_senales_es_opcional_entre_los_nodos(descripcion):
    opcionales = {n.node_package for n in nodos(descripcion) if n.condition is not None}
    assert opcionales == OPCIONALES


def test_la_camara_se_incluye_como_opcional(descripcion):
    incluidos = [e for e in descripcion.entities if isinstance(e, IncludeLaunchDescription)]
    assert len(incluidos) == 1
    assert incluidos[0].condition is not None


def test_las_opciones_tienen_los_valores_por_defecto_documentados(descripcion):
    opciones = {e.name: e for e in descripcion.entities if isinstance(e, DeclareLaunchArgument)}
    assert set(opciones) == {'camera', 'signs', 'serial_port'}
    assert opciones['camera'].default_value[0].text == 'true'
    assert opciones['signs'].default_value[0].text == 'false'


def test_el_puerto_serial_por_defecto_es_el_del_yaml_del_puente(descripcion):
    ruta = Path(get_package_share_directory('bfmc_hardware_bridge')) / 'config' / 'params.yaml'
    esperado = yaml.safe_load(ruta.read_text(encoding='utf-8'))[
        'hardware_bridge']['ros__parameters']['serial_port']
    opciones = {e.name: e for e in descripcion.entities if isinstance(e, DeclareLaunchArgument)}
    assert opciones['serial_port'].default_value[0].text == esperado


def test_cada_paquete_lanzado_esta_declarado_como_dependencia():
    package_xml = Path(__file__).resolve().parents[1] / 'package.xml'
    dependencias = {d.text for d in ET.parse(package_xml).getroot().iter('exec_depend')}
    assert set(NODOS) | {'bfmc_camera'} <= dependencias
