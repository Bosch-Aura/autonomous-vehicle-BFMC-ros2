"""Tests del resumen de latencias (medir_latencia.py), sin ROS."""
from bfmc_bringup.medir_latencia import resumen
import pytest


def test_el_resumen_da_cantidad_promedio_p99_y_maximo():
    latencias = [0.010] * 99 + [0.200]
    datos = resumen(latencias)
    assert datos['cantidad'] == 100
    assert datos['promedio_s'] == pytest.approx((0.010 * 99 + 0.200) / 100)
    assert datos['p99_s'] == pytest.approx(0.010)       # el 99 % queda en 10 ms
    assert datos['maximo_s'] == pytest.approx(0.200)


def test_con_una_sola_medicion_todo_es_esa_medicion():
    assert resumen([0.05]) == {'cantidad': 1, 'promedio_s': 0.05, 'p99_s': 0.05,
                               'maximo_s': 0.05}


def test_sin_mediciones_avisa():
    with pytest.raises(ValueError):
        resumen([])
