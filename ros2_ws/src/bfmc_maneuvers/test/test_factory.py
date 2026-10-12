"""Tests de la fábrica de estrategias."""
from bfmc_maneuvers.strategies.factory import build_strategy, ManeuverParams
from bfmc_maneuvers.strategies.intersection import (
    IntersectionParams, IntersectionStrategy, TurnPlan,
)
from bfmc_maneuvers.strategies.stop import StopParams, StopStrategy
import pytest

PARAMS = ManeuverParams(
    stop=StopParams(hold_s=3.0, exit_s=1.5, exit_speed_mps=0.25),
    intersection=IntersectionParams(
        speed_mps=0.25, straight_s=3.0,
        left=TurnPlan(approach_s=1.0, turn_s=2.0, steer_rad=0.375),
        right=TurnPlan(approach_s=1.5, turn_s=3.5, steer_rad=0.5)),
)


def test_el_nombre_stop_crea_la_estrategia_de_stop():
    assert isinstance(build_strategy('stop', '', PARAMS), StopStrategy)


def test_el_stop_ignora_la_direccion():
    assert isinstance(build_strategy('stop', 'left', PARAMS), StopStrategy)


@pytest.mark.parametrize('direccion', ['left', 'right', 'straight'])
def test_la_interseccion_se_crea_con_cada_direccion_valida(direccion):
    assert isinstance(build_strategy('intersection', direccion, PARAMS), IntersectionStrategy)


@pytest.mark.parametrize('direccion', ['', 'atras'])
def test_la_interseccion_sin_direccion_valida_se_rechaza(direccion):
    with pytest.raises(ValueError):
        build_strategy('intersection', direccion, PARAMS)


@pytest.mark.parametrize('nombre', ['', 'parking', 'STOP'])
def test_una_maniobra_desconocida_se_rechaza(nombre):
    with pytest.raises(ValueError):
        build_strategy(nombre, '', PARAMS)


def test_cada_pedido_crea_una_estrategia_nueva_con_su_propio_estado():
    primera = build_strategy('stop', '', PARAMS)
    segunda = build_strategy('stop', '', PARAMS)
    primera.start(now=0.0)
    with pytest.raises(RuntimeError):
        segunda.tick(now=0.0)
