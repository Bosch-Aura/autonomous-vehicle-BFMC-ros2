"""Tests de que la configuración del modelo y de las clases sea coherente.

No descargan nada (el CI no usa la red). Que el checksum declarado sea el del
archivo publicado en el release se verificó a mano al publicarlo y se vuelve a
verificar al ejecutar `descargar_modelo`, que rechaza un archivo con otro checksum.
"""
from pathlib import Path

import pytest
import yaml

from bfmc_sign_detection.model_download import cargar_release
from bfmc_sign_detection.sign_detection_logic import parse_class_map

CONFIG = Path(__file__).resolve().parents[1] / 'config'

# Las 13 clases que tiene el modelo, leídas de los metadatos de su archivo.
CLASES_DEL_MODELO = {
    'crosswalk', 'highway_entry', 'highway_exit', 'lightgreen', 'lightred', 'lightyellow',
    'no_entry', 'onewayroad', 'parking', 'priority', 'roundabout', 'stop', 'trafficlight',
}
# Las etiquetas que consume bfmc_decision.
ETIQUETAS_QUE_USA_DECISION = {'stop', 'priority', 'roundabout'}


@pytest.fixture(scope='module')
def parametros():
    with open(CONFIG / 'params.yaml', encoding='utf-8') as archivo:
        return yaml.safe_load(archivo)['sign_detection']['ros__parameters']


def test_el_release_declarado_es_valido():
    release = cargar_release(CONFIG / 'model.yaml')
    assert release.repository == 'Bosch-Aura/autonomous-vehicle-BFMC-ros2'
    assert release.asset == 'best.pt'
    assert len(release.sha256) == 64


def test_el_mapa_de_clases_cubre_las_trece_clases_del_modelo(parametros):
    assert set(parse_class_map(parametros['class_map'])) == CLASES_DEL_MODELO


def test_el_mapa_de_clases_produce_las_etiquetas_que_usa_decision(parametros):
    etiquetas = set(parse_class_map(parametros['class_map']).values())
    assert ETIQUETAS_QUE_USA_DECISION <= etiquetas


def test_cada_etiqueta_es_unica(parametros):
    etiquetas = list(parse_class_map(parametros['class_map']).values())
    assert len(etiquetas) == len(set(etiquetas))


def test_la_profundidad_que_se_lee_es_la_alineada_al_color(parametros):
    assert 'aligned_depth_to_color' in parametros['depth_topic']


def test_el_modelo_se_guarda_fuera_del_repositorio(parametros):
    assert parametros['model_path'].startswith('~/')
    assert parametros['model_path'].endswith('.pt')
