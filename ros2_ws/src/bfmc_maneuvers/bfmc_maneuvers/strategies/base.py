"""Contrato común de las maniobras (patrón Strategy).

Cada maniobra es una clase que implementa `ManeuverStrategy`. El nodo que las
ejecuta no sabe qué hace cada una: solo llama a `start` y a `tick`.
"""
from abc import ABC, abstractmethod
from dataclasses import dataclass


@dataclass(frozen=True)
class DriveOutput:
    """Comando de conducción en unidades SI."""

    speed: float
    """Velocidad en m/s."""

    steering: float
    """Ángulo de dirección en rad, positivo a la izquierda."""


@dataclass(frozen=True)
class StrategyStep:
    """Resultado de un ciclo de la maniobra."""

    output: DriveOutput
    finished: bool


class ManeuverStrategy(ABC):
    """Una maniobra puntual.

    No lee el reloj: recibe el instante actual en cada llamada, así una maniobra
    de varios segundos se testea en milisegundos.
    """

    @abstractmethod
    def start(self, now: float) -> None:
        """Inicia la maniobra en el instante `now` (segundos)."""

    @abstractmethod
    def tick(self, now: float) -> StrategyStep:
        """Avanza un ciclo y devuelve el comando y si la maniobra terminó."""
