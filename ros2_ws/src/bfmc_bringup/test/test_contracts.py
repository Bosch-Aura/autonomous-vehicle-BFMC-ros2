"""Tests de coherencia entre los params.yaml de todos los paquetes.

Cada paquete declara los nombres de los topics que usa en su propio YAML, y ningún
paquete conoce a los demás. Si dos paquetes que deben hablarse tienen nombres
distintos, el sistema arranca pero no se comunica, sin ningún error. Estos tests
comparan los nombres y las etiquetas entre los YAML y lo detectan antes de llegar
al auto.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import pytest
import yaml


def parametros(paquete, nodo):
    ruta = Path(get_package_share_directory(paquete)) / 'config' / 'params.yaml'
    with open(ruta, encoding='utf-8') as archivo:
        return yaml.safe_load(archivo)[nodo]['ros__parameters']


@pytest.fixture(scope='module')
def p():
    return {
        'camera': parametros('bfmc_camera', 'camera'),
        'lane': parametros('bfmc_lane_detection', 'lane_detection'),
        'signs': parametros('bfmc_sign_detection', 'sign_detection'),
        'control': parametros('bfmc_control', 'control'),
        'bridge': parametros('bfmc_hardware_bridge', 'hardware_bridge'),
        'maneuvers': parametros('bfmc_maneuvers', 'maneuvers'),
        'decision': parametros('bfmc_decision', 'decision'),
    }


# Cada contrato: (quien publica, parámetro) -> [(quien consume, parámetro), ...]
# Un nombre de topic o de acción tiene que ser el mismo en todos los YAML que lo mencionan.
CONTRATOS = {
    'estado del carril': [('lane', 'lane_topic'), ('control', 'lane_topic'),
                          ('decision', 'lane_topic')],
    'señales detectadas': [('signs', 'signs_topic'), ('decision', 'signs_topic')],
    'comando de carril': [('control', 'cmd_topic'), ('decision', 'lane_cmd_topic')],
    'estado del hardware': [('bridge', 'status_topic'), ('decision', 'hardware_topic')],
    'comando final': [('decision', 'drive_topic'), ('bridge', 'cmd_topic')],
    'comando de maniobra': [('maneuvers', 'cmd_topic'), ('decision', 'maneuver_cmd_topic')],
    'acción de maniobras': [('maneuvers', 'action_name'),
                            ('decision', 'maneuver_action_name')],
    'imagen de color': [('lane', 'image_topic'), ('signs', 'color_topic')],
}


@pytest.mark.parametrize('contrato', sorted(CONTRATOS))
def test_los_paquetes_que_se_comunican_usan_el_mismo_nombre(p, contrato):
    nombres = {(quien, clave): p[quien][clave] for quien, clave in CONTRATOS[contrato]}
    assert len(set(nombres.values())) == 1, 'nombres distintos: %s' % nombres


def test_el_topic_de_profundidad_de_las_senales_es_el_alineado_que_publica_la_camara(p):
    # La cámara publica la profundidad alineada con su nombre por defecto en el driver.
    assert p['signs']['depth_topic'] == '/camera/aligned_depth_to_color/image_raw'
    assert p['camera']['align_depth.enable'] is True


def test_el_topic_de_color_es_el_que_publica_la_camara(p):
    # Sin espacio de nombres y con camera_name 'camera', el driver publica /camera/color/...
    assert p['camera']['camera_name'] == 'camera'
    assert p['lane']['image_topic'] == '/camera/color/image_raw'


def test_solo_decision_publica_el_comando_final(p):
    comando_final = p['decision']['drive_topic']
    otros = {
        'control': p['control']['cmd_topic'],
        'maneuvers': p['maneuvers']['cmd_topic'],
    }
    assert all(topic != comando_final for topic in otros.values()), otros


def test_las_etiquetas_de_senal_que_usa_decision_las_produce_el_detector(p):
    etiquetas = {entrada.partition(':')[2].strip() for entrada in p['signs']['class_map']}
    usadas = set(p['decision']['stop_labels']) | set(p['decision']['intersection_labels'])
    assert usadas <= etiquetas, 'decision espera etiquetas que el detector no produce: %s' % (
        usadas - etiquetas)


def test_las_maniobras_que_pide_decision_existen_en_el_servidor(p):
    # decision pide 'stop' e 'intersection'; el servidor define parámetros para ambas.
    assert 'stop.hold_s' in _planos(p['maneuvers'])
    assert 'intersection.speed_mps' in _planos(p['maneuvers'])


def test_la_direccion_de_interseccion_que_pide_decision_es_valida_para_el_servidor(p):
    assert p['decision']['intersection_direction'] in ('left', 'right', 'straight')


def test_el_limite_de_velocidad_del_puente_cubre_la_velocidad_de_crucero_del_control(p):
    assert p['control']['cruise_speed_mps'] <= p['bridge']['max_speed_mps']


def test_el_limite_de_giro_del_puente_cubre_el_maximo_del_control(p):
    assert p['control']['max_steer_rad'] <= p['bridge']['max_steer_rad'] + 1e-3


def test_las_velocidades_de_las_maniobras_no_superan_el_limite_del_puente(p):
    velocidades = [p['maneuvers']['stop']['exit_speed_mps'],
                   p['maneuvers']['intersection']['speed_mps']]
    assert max(velocidades) <= p['bridge']['max_speed_mps']


def test_el_giro_de_las_maniobras_no_supera_el_limite_del_puente(p):
    giros = [p['maneuvers']['intersection'][lado]['steer_rad'] for lado in ('left', 'right')]
    assert max(giros) <= p['bridge']['max_steer_rad']


def test_las_entradas_vencidas_de_decision_toleran_la_frecuencia_con_que_se_publican(p):
    # El estado del hardware se publica a status_rate_hz: su plazo debe ser mayor que el período.
    periodo_hardware = 1.0 / p['bridge']['status_rate_hz']
    assert p['decision']['hardware_timeout_s'] > periodo_hardware


def test_el_plazo_del_comando_del_puente_es_mayor_que_el_periodo_de_decision(p):
    assert p['bridge']['cmd_timeout_s'] > 1.0 / p['decision']['rate_hz']


def _planos(diccionario, prefijo=''):
    """Aplana un diccionario anidado: {'a': {'b': 1}} -> {'a.b': 1}."""
    resultado = {}
    for clave, valor in diccionario.items():
        nombre = prefijo + clave
        if isinstance(valor, dict):
            resultado.update(_planos(valor, nombre + '.'))
        else:
            resultado[nombre] = valor
    return resultado
