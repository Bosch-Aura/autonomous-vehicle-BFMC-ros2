"""Interfaz común de los algoritmos de giro y lo que comparten.

Cada algoritmo recibe el estado del carril y devuelve el giro objetivo y la velocidad. Lo
demás (qué hacer sin carril, la saturación, el filtro y el límite de velocidad de giro de la
salida, y el límite físico de la curva) lo hace ControlLogic igual para todos: así, al
comparar dos algoritmos, la única diferencia es el algoritmo.

Para agregar uno: una clase que herede de AlgoritmoDeGiro en un archivo de esta carpeta, una
línea en REGISTRO (algoritmos/__init__.py) y su nombre en tipos.ALGORITMOS.
"""
from abc import ABC, abstractmethod
from collections import deque
import math
from typing import Tuple

from bfmc_control.tipos import ControlConfig, LaneInput


def limitar(valor: float, maximo: float) -> float:
    """Recorta `valor` a [-maximo, maximo]."""
    return max(-maximo, min(maximo, valor))


class CurvaturaConfiable:
    """La curvatura del carril que se puede usar, con su historia.

    Con una sola línea la curvatura medida es ruido (desvío estándar 1,24 1/m contra 0,41 con
    dos, grabación del 2026-11-08), así que solo se toma con confianza alta
    (`curvature_ff_min_confidence`) y se limita a la curva más cerrada de la pista
    (`curvature_ff_max`). Entre mediciones confiables se sostiene la última durante
    `curvature_ff_hold_s`. Guarda las muestras para poder usarlas con retraso.
    """

    def __init__(self):
        self.reiniciar()

    def reiniciar(self):
        """Olvida todo lo medido."""
        self._valor = 0.0
        self._instante = None
        self.muestras = deque(maxlen=512)    # (instante, curvatura), de la más vieja a la nueva

    def actualizar(self, cfg: ControlConfig, lane: LaneInput, now: float) -> float:
        """Toma la medición si es confiable y devuelve la curvatura vigente (0 si no hay)."""
        if self.muestras and self.muestras[-1][0] > now:
            self.reiniciar()    # el reloj volvió atrás (otra grabación)
        if lane.confidence >= cfg.curvature_ff_min_confidence and math.isfinite(lane.curvature):
            self._valor = limitar(lane.curvature, cfg.curvature_ff_max)
            self._instante = now
            self.muestras.append((now, self._valor))
        elif (self._instante is None
              or not 0.0 <= now - self._instante <= cfg.curvature_ff_hold_s):
            return 0.0
        return self._valor


class AlgoritmoDeGiro(ABC):
    """Un algoritmo de dirección: del estado del carril al giro objetivo y la velocidad."""

    def __init__(self):
        self.curvatura = CurvaturaConfiable()

    def reiniciar(self) -> None:
        """Olvida el estado interno (el algoritmo arranca como recién creado)."""
        self.curvatura.reiniciar()

    def al_perder_el_carril(self) -> None:
        """Se llama en cada ciclo sin carril usable. Por defecto no hace nada."""

    @abstractmethod
    def calcular(self, cfg: ControlConfig, lane: LaneInput, now: float) -> Tuple[float, float]:
        """Devuelve (giro objetivo [rad], velocidad [m/s]) con el carril usable.

        El giro todavía no está filtrado ni limitado: eso lo hace ControlLogic. Puede pasarse
        del tope del servo; ControlLogic lo satura.
        """
