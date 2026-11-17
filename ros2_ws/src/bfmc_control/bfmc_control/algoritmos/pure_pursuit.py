"""Pure Pursuit sobre el estado del carril (A2-155), con Ld que se acorta en las curvas.

Las cuentas (punto objetivo, ley de control, velocidad) están en bfmc_control/pure_pursuit.py;
esta clase las usa con la curvatura confiable (la misma que el término de curvatura del PID:
con una sola línea la curvatura es ruido) y elige la distancia de mirada.

Ld en las curvas: con Ld fija en 0,70 m y la curva más cerrada de ~0,46 m de radio, el punto
objetivo cae más lejos que el radio y la parábola del carril ya no representa la curva: el
auto corta camino o gira de menos. Con `lookahead_curve_m` > 0, Ld baja de la de recta a esa
a medida que la curvatura llega a `lookahead_curve_kappa` (la idea de VROOM, BFMC 2024, de
ajustar Pure Pursuit con la curvatura del carril).
"""
from bfmc_control.algoritmos.base import AlgoritmoDeGiro
from bfmc_control.pure_pursuit import (
    angulo_de_direccion, distancia_de_mirada, punto_objetivo, velocidad,
)


def mirada_segun_curvatura(pp, ld_recta: float, curvatura: float) -> float:
    """Ld interpolada entre la de recta y `lookahead_curve_m` según |curvatura|."""
    if pp.lookahead_curve_m <= 0.0 or pp.lookahead_curve_m >= ld_recta:
        return ld_recta
    fraccion = min(1.0, abs(curvatura) / pp.lookahead_curve_kappa)
    return ld_recta - fraccion * (ld_recta - pp.lookahead_curve_m)


class PurePursuit(AlgoritmoDeGiro):
    """Apunta a un punto del centro del carril a Ld del eje trasero."""

    def calcular(self, cfg, lane, now):
        pp = cfg.pure_pursuit
        curvatura = self.curvatura.actualizar(cfg, lane, now)
        rapidez = velocidad(pp, cfg.cruise_speed_mps, curvatura, lane.confidence)
        ld = mirada_segun_curvatura(pp, distancia_de_mirada(pp, rapidez), curvatura)
        x, y = punto_objetivo(pp, lane.lateral_error_m, lane.heading_error_rad,
                              curvatura, lane.reference_distance_m, ld)
        return angulo_de_direccion(pp, x, y), rapidez
