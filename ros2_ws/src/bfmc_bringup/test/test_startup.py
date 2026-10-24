"""Prueba de arranque: lanza el sistema completo y comprueba que quede en el estado seguro.

Arranca todos los nodos de verdad, con los params.yaml reales, sin cámara ni
señales, y sin ESP32 conectado. Con ninguna entrada, el sistema tiene que quedar
detenido: `bfmc_decision` en STARTING y el comando final en cero.

Usa un dominio de ROS propio para no cruzarse con los tests de otros paquetes que
corren a la vez.
"""
import os
import signal
import subprocess
import time

import pytest

NODOS = ['/lane_detection', '/control', '/hardware_bridge', '/decision', '/maneuvers']
PLAZO_DE_ARRANQUE_S = 40.0


@pytest.fixture(scope='module')
def entorno():
    entorno = dict(os.environ)
    entorno['ROS_DOMAIN_ID'] = str(70 + os.getpid() % 25)
    entorno['ROS_LOCALHOST_ONLY'] = '1'
    return entorno


@pytest.fixture(scope='module')
def sistema(entorno):
    proceso = subprocess.Popen(
        ['ros2', 'launch', 'bfmc_bringup', 'system.launch.py', 'camera:=false', 'signs:=false'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=entorno,
        start_new_session=True)
    yield proceso
    if proceso.poll() is None:
        os.killpg(proceso.pid, signal.SIGINT)
        try:
            proceso.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(proceso.pid, signal.SIGKILL)


def ros2(entorno, *argumentos, plazo=20):
    resultado = subprocess.run(
        ['ros2', *argumentos], capture_output=True, text=True, env=entorno, timeout=plazo)
    return resultado.stdout


def esperar_nodos(entorno):
    limite = time.monotonic() + PLAZO_DE_ARRANQUE_S
    visibles = set()
    while time.monotonic() < limite:
        visibles = set(ros2(entorno, 'node', 'list').split())
        if set(NODOS) <= visibles:
            return visibles
        time.sleep(1.0)
    return visibles


def test_arrancan_todos_los_nodos_del_sistema(sistema, entorno):
    visibles = esperar_nodos(entorno)
    assert set(NODOS) <= visibles, 'faltan nodos: %s' % (set(NODOS) - visibles)


def test_ningun_proceso_del_sistema_murio_al_arrancar(sistema, entorno):
    esperar_nodos(entorno)
    time.sleep(3.0)
    assert sistema.poll() is None, 'el launch terminó'
    # Los nodos siguen vivos tras unos segundos: ninguno murió al construirse.
    assert set(NODOS) <= set(ros2(entorno, 'node', 'list').split())


def test_sin_entradas_decision_queda_en_el_estado_de_arranque(sistema, entorno):
    esperar_nodos(entorno)
    salida = ros2(entorno, 'topic', 'echo', '/vehicle/state', '--once', '--field', 'state')
    assert salida.split()[0] == '0'


def test_sin_entradas_el_comando_final_es_cero(sistema, entorno):
    esperar_nodos(entorno)
    salida = ros2(entorno, 'topic', 'echo', '/cmd/drive', '--once', '--field', 'drive.speed')
    assert float(salida.split()[0]) == 0.0


def test_el_sistema_expone_los_topics_de_la_arquitectura(sistema, entorno):
    esperar_nodos(entorno)
    topics = set(ros2(entorno, 'topic', 'list').split())
    esperados = {'/cmd/drive', '/vehicle/state', '/hardware/status', '/perception/lane',
                 '/control/lane_cmd'}
    assert esperados <= topics, 'faltan topics: %s' % (esperados - topics)


def test_el_servidor_de_maniobras_esta_disponible(sistema, entorno):
    esperar_nodos(entorno)
    assert '/maneuver/execute' in ros2(entorno, 'action', 'list').split()
