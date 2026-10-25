"""Maniobras como una secuencia de tramos con velocidad y dirección constantes.

Las maniobras de este paquete (stop, intersección) son secuencias en lazo abierto:
cada tramo dura un tiempo y mantiene un comando. Este módulo resuelve la parte
común, así cada maniobra solo declara sus tramos. No lee el reloj: recibe el
instante actual en cada llamada.
"""
from dataclasses import dataclass
import math
from typing import Optional, Sequence

from bfmc_maneuvers.strategies.base import DriveOutput, ManeuverStrategy, StrategyStep


@dataclass(frozen=True)
class Leg:
    """Un tramo de la maniobra."""

    duration_s: float
    """Duración en segundos."""

    speed_mps: float
    """Velocidad en m/s. Positivo avanza."""

    steering_rad: float
    """Ángulo de dirección en rad. Positivo a la izquierda."""

    def __post_init__(self):
        if not all(math.isfinite(v) for v in (self.duration_s, self.speed_mps, self.steering_rad)):
            raise ValueError('un tramo no puede tener valores NaN ni infinitos')
        if self.duration_s < 0.0:
            raise ValueError('la duración de un tramo no puede ser negativa: %s' % self.duration_s)
        if self.speed_mps < 0.0:
            raise ValueError('la velocidad de un tramo no puede ser negativa: %s' % self.speed_mps)


class LegSequence:
    """Una lista de tramos y qué tramo corresponde a cada tiempo transcurrido."""

    def __init__(self, legs: Sequence[Leg]):
        self._legs = [leg for leg in legs if leg.duration_s > 0.0]
        self._ends = []
        total = 0.0
        for leg in self._legs:
            total += leg.duration_s
            self._ends.append(total)
        self.total_s = total

    def at(self, elapsed_s: float) -> Optional[Leg]:
        """El tramo vigente tras `elapsed_s` segundos, o None si la secuencia terminó."""
        for leg, end in zip(self._legs, self._ends):
            if elapsed_s < end:
                return leg
        return None


class LegStrategy(ManeuverStrategy):
    """Ejecuta una secuencia de tramos y termina al completarla."""

    def __init__(self, legs: Sequence[Leg]):
        self._sequence = LegSequence(legs)
        self._started_at = None

    def start(self, now: float) -> None:
        self._started_at = now

    def tick(self, now: float) -> StrategyStep:
        if self._started_at is None:
            raise RuntimeError('tick() se llamó antes de start()')
        leg = self._sequence.at(now - self._started_at)
        if leg is None:
            return StrategyStep(output=DriveOutput(speed=0.0, steering=0.0), finished=True)
        return StrategyStep(
            output=DriveOutput(speed=leg.speed_mps, steering=leg.steering_rad), finished=False)
