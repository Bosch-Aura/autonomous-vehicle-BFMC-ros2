"""Tests de la maniobra de intersección, con el tiempo simulado."""
import pytest

from bfmc_maneuvers.strategies.intersection import (
    IntersectionParams, IntersectionStrategy, TurnPlan,
)

PARAMS = IntersectionParams(
    speed_mps=0.25, straight_s=3.0,
    left=TurnPlan(approach_s=1.0, turn_s=2.0, steer_rad=0.375),
    right=TurnPlan(approach_s=1.5, turn_s=3.5, steer_rad=0.5),
)


def intersection(direccion):
    strategy = IntersectionStrategy(direccion, PARAMS)
    strategy.start(now=0.0)
    return strategy


# --- giro a la izquierda ------------------------------------------------------------------

def test_a_la_izquierda_primero_avanza_derecho():
    paso = intersection('left').tick(now=0.5)
    assert (paso.output.speed, paso.output.steering) == (0.25, 0.0)


def test_a_la_izquierda_despues_gira_con_direccion_positiva():
    paso = intersection('left').tick(now=1.0)
    assert (paso.output.speed, paso.output.steering) == (0.25, 0.375)
    assert paso.finished is False


def test_a_la_izquierda_termina_tras_el_avance_y_el_giro():
    estrategia = intersection('left')
    assert estrategia.tick(now=2.875).finished is False
    assert estrategia.tick(now=3.0).finished is True


# --- giro a la derecha ---------------------------------------------------------------------

def test_a_la_derecha_gira_con_direccion_negativa():
    paso = intersection('right').tick(now=1.5)
    assert (paso.output.speed, paso.output.steering) == (0.25, -0.5)


def test_a_la_derecha_usa_sus_propios_tiempos():
    estrategia = intersection('right')
    assert estrategia.tick(now=4.875).finished is False
    assert estrategia.tick(now=5.0).finished is True


def test_el_sentido_lo_da_la_direccion_aunque_el_parametro_venga_con_signo():
    params = IntersectionParams(
        speed_mps=0.25, straight_s=3.0,
        left=TurnPlan(approach_s=0.0, turn_s=1.0, steer_rad=-0.375),
        right=TurnPlan(approach_s=0.0, turn_s=1.0, steer_rad=-0.5))
    izquierda = IntersectionStrategy('left', params)
    derecha = IntersectionStrategy('right', params)
    izquierda.start(now=0.0)
    derecha.start(now=0.0)
    assert izquierda.tick(now=0.5).output.steering == 0.375
    assert derecha.tick(now=0.5).output.steering == -0.5


# --- seguir derecho -----------------------------------------------------------------------

def test_seguir_derecho_no_gira_en_ningun_momento():
    estrategia = intersection('straight')
    for instante in (0.0, 1.5, 2.875):
        paso = estrategia.tick(now=instante)
        assert (paso.output.speed, paso.output.steering, paso.finished) == (0.25, 0.0, False)


def test_seguir_derecho_dura_lo_configurado():
    estrategia = intersection('straight')
    assert estrategia.tick(now=3.0).finished is True


# --- validación -----------------------------------------------------------------------------

@pytest.mark.parametrize('direccion', ['', 'atras', 'LEFT', 'izquierda'])
def test_una_direccion_desconocida_se_rechaza(direccion):
    with pytest.raises(ValueError):
        IntersectionStrategy(direccion, PARAMS)
