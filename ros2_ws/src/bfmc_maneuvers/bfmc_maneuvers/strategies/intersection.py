"""Maniobra de intersección: cruzar en lazo abierto, girando o siguiendo derecho."""
from dataclasses import dataclass

from bfmc_maneuvers.strategies.legs import Leg, LegStrategy

DIRECTIONS = ('left', 'right', 'straight')


@dataclass(frozen=True)
class TurnPlan:
    """Cómo se hace un giro: avanzar derecho `approach_s` y girar `turn_s` con `steer_rad`."""

    approach_s: float
    turn_s: float
    steer_rad: float
    """Magnitud del ángulo de dirección, siempre positiva: el sentido lo da la dirección."""


@dataclass(frozen=True)
class IntersectionParams:
    """Parámetros de la intersección. Sus valores vienen de config/params.yaml."""

    speed_mps: float
    straight_s: float
    left: TurnPlan
    right: TurnPlan


class IntersectionStrategy(LegStrategy):
    """Cruza una intersección en la dirección pedida.

    En lazo abierto: dentro de la intersección las líneas del carril desaparecen,
    así que no se puede seguir el carril y se ejecuta una secuencia por tiempo.
    """

    def __init__(self, direction: str, params: IntersectionParams):
        if direction not in DIRECTIONS:
            raise ValueError('dirección desconocida: "%s". Válidas: %s'
                             % (direction, ', '.join(DIRECTIONS)))
        speed = params.speed_mps

        if direction == 'straight':
            legs = [Leg(duration_s=params.straight_s, speed_mps=speed, steering_rad=0.0)]
        else:
            plan = params.left if direction == 'left' else params.right
            sentido = 1.0 if direction == 'left' else -1.0
            legs = [
                Leg(duration_s=plan.approach_s, speed_mps=speed, steering_rad=0.0),
                Leg(duration_s=plan.turn_s, speed_mps=speed,
                    steering_rad=sentido * abs(plan.steer_rad)),
            ]
        super().__init__(legs)
