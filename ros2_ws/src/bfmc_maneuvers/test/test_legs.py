"""Tests de la secuencia de tramos: sin ROS y con el tiempo simulado."""
import math

from bfmc_maneuvers.strategies.legs import Leg, LegSequence, LegStrategy
import pytest

A = Leg(duration_s=1.0, speed_mps=0.5, steering_rad=0.0)
B = Leg(duration_s=2.0, speed_mps=0.25, steering_rad=0.5)


# --- qué tramo corresponde a cada instante ----------------------------------------------

def test_al_comenzar_corresponde_el_primer_tramo():
    assert LegSequence([A, B]).at(0.0) == A


def test_justo_al_terminar_un_tramo_empieza_el_siguiente():
    assert LegSequence([A, B]).at(1.0) == B


def test_dentro_del_ultimo_tramo_sigue_vigente():
    assert LegSequence([A, B]).at(2.75) == B


def test_al_cumplirse_la_duracion_total_la_secuencia_termina():
    assert LegSequence([A, B]).at(3.0) is None


def test_la_duracion_total_es_la_suma_de_los_tramos():
    assert LegSequence([A, B]).total_s == pytest.approx(3.0)


def test_un_tramo_de_duracion_cero_se_omite():
    corto = Leg(duration_s=0.0, speed_mps=9.0, steering_rad=0.0)
    secuencia = LegSequence([corto, A])
    assert secuencia.at(0.0) == A
    assert secuencia.total_s == pytest.approx(1.0)


def test_una_secuencia_vacia_termina_de_inmediato():
    assert LegSequence([]).at(0.0) is None


# --- validación de los tramos -----------------------------------------------------------

@pytest.mark.parametrize('cambio', [
    {'duration_s': -1.0},
    {'speed_mps': -0.1},
    {'duration_s': math.nan},
    {'speed_mps': math.inf},
    {'steering_rad': math.nan},
])
def test_un_tramo_invalido_se_rechaza(cambio):
    base = {'duration_s': 1.0, 'speed_mps': 0.5, 'steering_rad': 0.0}
    with pytest.raises(ValueError):
        Leg(**{**base, **cambio})


# --- la estrategia sobre la secuencia ------------------------------------------------------

def estrategia():
    strategy = LegStrategy([A, B])
    strategy.start(now=100.0)
    return strategy


def test_durante_cada_tramo_se_entrega_su_comando():
    strategy = estrategia()
    paso = strategy.tick(now=100.5)
    assert (paso.output.speed, paso.output.steering, paso.finished) == (0.5, 0.0, False)
    paso = strategy.tick(now=101.5)
    assert (paso.output.speed, paso.output.steering, paso.finished) == (0.25, 0.5, False)


def test_al_completar_la_secuencia_termina_con_el_auto_detenido():
    paso = estrategia().tick(now=103.0)
    assert paso.finished is True
    assert (paso.output.speed, paso.output.steering) == (0.0, 0.0)


def test_una_maniobra_de_tres_segundos_se_recorre_sin_esperar():
    strategy = estrategia()
    ciclos = 0
    while not strategy.tick(now=100.0 + ciclos * 0.125).finished:
        ciclos += 1
    assert ciclos == 24


def test_llamar_a_tick_sin_start_es_un_error():
    with pytest.raises(RuntimeError):
        LegStrategy([A]).tick(now=0.0)


def test_se_puede_reiniciar_la_maniobra_con_un_nuevo_start():
    strategy = estrategia()
    assert strategy.tick(now=104.0).finished is True
    strategy.start(now=200.0)
    assert strategy.tick(now=200.5).finished is False
