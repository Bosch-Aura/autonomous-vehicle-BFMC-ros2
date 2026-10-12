"""Ejemplo de test unitario con tiempo simulado.

Muestra la regla de diseño que hace testeable al sistema: la lógica no lee el
reloj ni duerme, recibe el instante actual (`now`) como argumento. Así una
espera de 8 segundos se verifica en milisegundos.

La clase `Watchdog` está acá para que el ejemplo sea autocontenido. En el
proyecto la lógica vive en su paquete y el test la importa.

Cómo correrlo (no necesita ROS):

    pytest .claude/skills/escribir-tests/ejemplos/test_unitario_tiempo_simulado.py
"""
import pytest


class Watchdog:
    """Se vence cuando pasan `timeout_s` segundos sin recibir `feed`."""

    def __init__(self, timeout_s: float):
        if timeout_s <= 0.0:
            raise ValueError('timeout_s debe ser positivo, recibido: %s' % timeout_s)
        self._timeout_s = timeout_s
        self._last_feed = None

    def feed(self, now: float) -> None:
        self._last_feed = now

    def expired(self, now: float) -> bool:
        if self._last_feed is None:
            return False
        return (now - self._last_feed) >= self._timeout_s


# Los instantes de estos tests son potencias de 2 (0.5, 0.125...): se representan
# exactos en punto flotante y evitan comparaciones inestables como 100.3 - 100.0.
TIMEOUT_S = 0.5


@pytest.fixture
def watchdog():
    watchdog = Watchdog(timeout_s=TIMEOUT_S)
    watchdog.feed(now=10.0)
    return watchdog


def test_antes_del_tiempo_limite_no_se_vence(watchdog):
    assert watchdog.expired(now=10.0 + TIMEOUT_S - 0.125) is False


def test_al_cumplirse_el_tiempo_limite_se_vence(watchdog):
    assert watchdog.expired(now=10.0 + TIMEOUT_S) is True


def test_recibir_un_dato_reinicia_la_cuenta(watchdog):
    watchdog.feed(now=10.25)
    assert watchdog.expired(now=10.25 + TIMEOUT_S - 0.125) is False


def test_sin_ningun_dato_todavia_no_se_considera_vencido():
    assert Watchdog(timeout_s=TIMEOUT_S).expired(now=1000.0) is False


def test_un_tiempo_limite_no_positivo_se_rechaza():
    with pytest.raises(ValueError):
        Watchdog(timeout_s=0.0)


def test_una_espera_de_ocho_segundos_se_verifica_sin_esperar():
    """Recorre 8 segundos simulados en pasos de 0.125 s: son 64 ciclos."""
    watchdog = Watchdog(timeout_s=8.0)
    watchdog.feed(now=0.0)

    ciclos = 0
    while not watchdog.expired(now=ciclos * 0.125):
        ciclos += 1

    assert ciclos == 64
