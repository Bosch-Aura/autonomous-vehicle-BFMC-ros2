"""Crea la estrategia que corresponde al nombre de una maniobra."""
from dataclasses import dataclass

from bfmc_maneuvers.strategies.base import ManeuverStrategy
from bfmc_maneuvers.strategies.intersection import (
    DIRECTIONS, IntersectionParams, IntersectionStrategy,
)
from bfmc_maneuvers.strategies.stop import StopParams, StopStrategy

MANEUVERS = ('stop', 'intersection')


@dataclass(frozen=True)
class ManeuverParams:
    """Parámetros de todas las maniobras."""

    stop: StopParams
    intersection: IntersectionParams


def build_strategy(maneuver: str, direction: str, params: ManeuverParams) -> ManeuverStrategy:
    """Devuelve una estrategia nueva, lista para `start`.

    Lanza ValueError si la maniobra o la dirección no son válidas. Agregar una
    maniobra nueva es agregar una clase y una rama acá: las demás no se tocan.
    """
    if maneuver == 'stop':
        return StopStrategy(params.stop)
    if maneuver == 'intersection':
        if direction not in DIRECTIONS:
            raise ValueError('la intersección necesita una dirección (%s), recibida: "%s"'
                             % (', '.join(DIRECTIONS), direction))
        return IntersectionStrategy(direction, params.intersection)
    raise ValueError('maniobra desconocida: "%s". Válidas: %s'
                     % (maneuver, ', '.join(MANEUVERS)))
