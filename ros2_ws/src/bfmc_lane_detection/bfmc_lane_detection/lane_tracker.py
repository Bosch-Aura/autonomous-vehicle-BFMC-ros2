"""Fallback escalonado del carril (A2-152). Lógica pura: sin ROS, el tiempo llega como argumento.

El detector (lane_detection_logic.py) mira cada imagen sola. Este módulo tiene la memoria:
recuerda el último estado válido y decide el modo.

- Dos líneas válidas: MODE_TWO_LINES.
- Una válida: MODE_ONE_LINE (el detector ya estimó el centro con medio ancho de carril).
- Ninguna, hace menos de `lost_timeout_s`: MODE_PREDICTED. Se repite el último estado
  válido y la confianza baja en línea recta hasta 0 al cumplirse el plazo. (Hasta que
  exista el Kalman de A2-154.)
- Ninguna por más de `lost_timeout_s`: MODE_LOST. `detected` es false, el desvío y el
  rumbo son NaN y la confianza 0.

Además guarda el polinomio de cada línea válida para que el detector busque cerca en el
cuadro siguiente (A2-153). Las reglas son por línea: el polinomio de una línea se olvida
si lleva más de `lost_timeout_s` sin ser válida (el mismo plazo: cuánto se le cree a una
medición vieja) o si el chequeo de ancho la descartó (lo encontrado cerca no era la
línea). Mientras tanto se sigue usando, así un hueco de una línea punteada no obliga a
buscar en toda la imagen.

Es el patrón común de degradación gradual: seguir con una estimación por un tiempo
acotado (el error de seguir a ciegas crece rápido) y después declarar el carril perdido
para que bfmc_decision lleve el auto a una parada controlada.

Los modos tienen los mismos números que las constantes de bfmc_interfaces/LaneState.
"""
from dataclasses import dataclass
import math
from typing import Optional

from bfmc_lane_detection.lane_detection_logic import LaneResult

MODE_TWO_LINES = 0
MODE_ONE_LINE = 1
MODE_PREDICTED = 2
MODE_LOST = 3
NOMBRES_DE_MODO = {MODE_TWO_LINES: 'DOS LINEAS', MODE_ONE_LINE: 'UNA LINEA',
                   MODE_PREDICTED: 'PREDICHO', MODE_LOST: 'PERDIDO'}


@dataclass(frozen=True)
class TrackedLane:
    """Lo que se publica en LaneState."""

    mode: int
    detected: bool
    lateral_error_m: float
    heading_error_rad: float
    confidence: float
    intersection_detected: bool
    left_valid: bool
    right_valid: bool


class LaneTracker:
    """Decide el modo con la medición de cada imagen y el instante de esa imagen."""

    def __init__(self, lost_timeout_s: float):
        self.lost_timeout_s = lost_timeout_s
        self._ultimo: Optional[LaneResult] = None
        self._instante_ultimo: Optional[float] = None
        # Por lado ('left', 'right'): (polinomio, instante en que fue válido).
        self._previas = {}

    def previas(self, now: float) -> tuple:
        """Los polinomios (izquierdo, derecho) vigentes en `now`, o None, para el detector."""
        return tuple(self._vigente(lado, now) for lado in ('left', 'right'))

    def olvidar_previas(self) -> None:
        """Olvida los polinomios: por ejemplo, si cambió la vista cenital (están en sus px)."""
        self._previas.clear()

    def _vigente(self, lado, now):
        guardada = self._previas.get(lado)
        if guardada is None:
            return None
        polinomio, instante = guardada
        return polinomio if 0.0 <= now - instante <= self._lost_timeout_s else None

    def _recordar_lineas(self, medicion: LaneResult, now: float) -> None:
        for lado, ajuste in (('left', medicion.left_fit), ('right', medicion.right_fit)):
            if ajuste is not None:
                self._previas[lado] = (ajuste, now)
            elif medicion.width_discarded == lado:
                self._previas.pop(lado, None)

    @property
    def lost_timeout_s(self) -> float:
        return self._lost_timeout_s

    @lost_timeout_s.setter
    def lost_timeout_s(self, valor: float) -> None:
        if not valor > 0.0:
            raise ValueError('lost_timeout_s debe ser mayor que 0')
        self._lost_timeout_s = valor

    def update(self, medicion: LaneResult, now: float) -> TrackedLane:
        """Devuelve el estado a publicar para la medición `medicion` tomada en `now` [s]."""
        self._recordar_lineas(medicion, now)
        if medicion.detected:
            self._ultimo, self._instante_ultimo = medicion, now
            modo = (MODE_TWO_LINES if medicion.left_valid and medicion.right_valid
                    else MODE_ONE_LINE)
            return self._estado(modo, medicion, medicion.confidence, medicion)

        transcurrido = None if self._ultimo is None else now - self._instante_ultimo
        # Un instante anterior al último válido (el reloj de la cámara volvió atrás) no
        # permite saber cuánto pasó: se toma como perdido, que es lo seguro.
        if transcurrido is None or not 0.0 <= transcurrido <= self._lost_timeout_s:
            return TrackedLane(MODE_LOST, False, math.nan, math.nan, 0.0,
                               medicion.intersection_detected, False, False)
        confianza = self._ultimo.confidence * (1.0 - transcurrido / self._lost_timeout_s)
        return self._estado(MODE_PREDICTED, self._ultimo, confianza, medicion)

    @staticmethod
    def _estado(modo, valores: LaneResult, confianza: float, actual: LaneResult):
        # La intersección y la validez de las líneas son siempre las de la imagen actual.
        return TrackedLane(modo, True, valores.lateral_error_m, valores.heading_error_rad,
                           confianza, actual.intersection_detected, actual.left_valid,
                           actual.right_valid)
