"""Maniobra stop: frenar, esperar y avanzar derecho para pasar la señal."""
from dataclasses import dataclass

from bfmc_maneuvers.strategies.legs import Leg, LegStrategy


@dataclass(frozen=True)
class StopParams:
    """Parámetros del stop. Sus valores vienen de config/params.yaml."""

    hold_s: float
    exit_s: float
    exit_speed_mps: float


class StopStrategy(LegStrategy):
    """Detiene el auto `hold_s` segundos y después avanza derecho `exit_s` segundos.

    El avance final saca al auto de la zona de la señal para que no la vuelva a
    detectar de inmediato. Con `exit_s` en 0 la maniobra termina al cumplirse la espera.
    """

    def __init__(self, params: StopParams):
        super().__init__([
            Leg(duration_s=params.hold_s, speed_mps=0.0, steering_rad=0.0),
            Leg(duration_s=params.exit_s, speed_mps=params.exit_speed_mps, steering_rad=0.0),
        ])
