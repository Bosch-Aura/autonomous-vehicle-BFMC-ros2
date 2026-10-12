"""Tests de la maniobra stop, con el tiempo simulado."""
import pytest

from bfmc_maneuvers.strategies.stop import StopParams, StopStrategy

PARAMS = StopParams(hold_s=3.0, exit_s=1.5, exit_speed_mps=0.25)


def stop(params=PARAMS):
    strategy = StopStrategy(params)
    strategy.start(now=10.0)
    return strategy


def test_durante_la_espera_el_auto_permanece_detenido():
    paso = stop().tick(now=12.875)
    assert (paso.output.speed, paso.output.steering, paso.finished) == (0.0, 0.0, False)


def test_al_cumplirse_la_espera_avanza_derecho_a_la_velocidad_de_salida():
    paso = stop().tick(now=13.0)
    assert (paso.output.speed, paso.output.steering, paso.finished) == (0.25, 0.0, False)


def test_al_terminar_el_avance_la_maniobra_finaliza_con_el_auto_detenido():
    paso = stop().tick(now=14.5)
    assert paso.finished is True
    assert (paso.output.speed, paso.output.steering) == (0.0, 0.0)


def test_sin_avance_final_la_maniobra_termina_al_cumplirse_la_espera():
    strategy = stop(StopParams(hold_s=3.0, exit_s=0.0, exit_speed_mps=0.25))
    assert strategy.tick(now=12.875).finished is False
    assert strategy.tick(now=13.0).finished is True


def test_un_stop_dura_lo_que_dice_su_configuracion():
    strategy = stop()
    ciclos = 0
    while not strategy.tick(now=10.0 + ciclos * 0.125).finished:
        ciclos += 1
    assert ciclos * 0.125 == pytest.approx(4.5)


def test_una_espera_negativa_se_rechaza():
    with pytest.raises(ValueError):
        StopStrategy(StopParams(hold_s=-1.0, exit_s=1.5, exit_speed_mps=0.25))
