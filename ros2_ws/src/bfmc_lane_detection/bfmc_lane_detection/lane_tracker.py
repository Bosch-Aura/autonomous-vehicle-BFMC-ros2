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

Con un filtro de Kalman (lane_kalman.py, A2-154; `kalman_enabled`), lo publicado es el
estado filtrado: en cada imagen predice con el avance del auto y corrige con la medición
(más peso con dos líneas que con una). En MODE_PREDICTED y MODE_LOST solo predice, y la
confianza baja con la incertidumbre del desvío. Se reinicia al salir de MODE_LOST con dos
líneas, y si descarta mediciones seguidas por más de `lost_timeout_s` (la realidad
cambió de golpe, por ejemplo después de una intersección: sin reiniciar, el filtro
quedaría aferrado a su estimación vieja). Sin filtro, se comporta como en A2-152.

Es el patrón común de degradación gradual: seguir con una estimación por un tiempo
acotado (el error de seguir a ciegas crece rápido) y después declarar el carril perdido
para que bfmc_decision lleve el auto a una parada controlada.

Los modos tienen los mismos números que las constantes de bfmc_interfaces/LaneState.
"""
from dataclasses import dataclass
import math
from typing import Optional

from bfmc_lane_detection.lane_detection_logic import LaneResult
from bfmc_lane_detection.lane_kalman import LaneKalman

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
    curvature: float
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
        self._modo_previo = MODE_LOST
        self._kalman: Optional[LaneKalman] = None
        self._max_std_m = 1.0
        self._instante_previo: Optional[float] = None
        self._rechazando_desde: Optional[float] = None
        # Por lado ('left', 'right'): (polinomio, instante en que fue válido).
        self._previas = {}

    def previas(self, now: float) -> tuple:
        """Los polinomios (izquierdo, derecho) vigentes en `now`, o None, para el detector."""
        return tuple(self._vigente(lado, now) for lado in ('left', 'right'))

    def olvidar_previas(self) -> None:
        """Olvida los polinomios y el filtro: por ejemplo, si cambió la vista cenital."""
        self._previas.clear()
        if self._kalman is not None:
            self._kalman.olvidar()

    def usar_kalman(self, kalman: Optional[LaneKalman], max_std_m: float = 1.0) -> None:
        """Activa el filtro (o lo saca con None). `max_std_m`: incertidumbre de confianza 0."""
        if max_std_m <= 0.0:
            raise ValueError('kalman_max_std_m debe ser positivo')
        self._kalman, self._max_std_m = kalman, max_std_m
        self._rechazando_desde = None

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

    def update(self, medicion: LaneResult, now: float,
               velocidad_mps: float = 0.0) -> TrackedLane:
        """Devuelve el estado a publicar para la medición `medicion` tomada en `now` [s].

        `velocidad_mps` es la del auto (la usa solo el filtro para predecir).
        """
        self._recordar_lineas(medicion, now)
        modo = self._modo(medicion, now)
        if self._kalman is None:
            salida = self._sin_filtro(modo, medicion, now)
        else:
            salida = self._con_filtro(modo, medicion, now, velocidad_mps)
        self._modo_previo = modo
        return salida

    def _modo(self, medicion: LaneResult, now: float) -> int:
        if medicion.detected:
            self._ultimo, self._instante_ultimo = medicion, now
            return (MODE_TWO_LINES if medicion.left_valid and medicion.right_valid
                    else MODE_ONE_LINE)
        transcurrido = None if self._ultimo is None else now - self._instante_ultimo
        # Un instante anterior al último válido (el reloj de la cámara volvió atrás) no
        # permite saber cuánto pasó: se toma como perdido, que es lo seguro.
        if transcurrido is None or not 0.0 <= transcurrido <= self._lost_timeout_s:
            return MODE_LOST
        return MODE_PREDICTED

    def _sin_filtro(self, modo, medicion: LaneResult, now: float) -> TrackedLane:
        if modo == MODE_LOST:
            return self._perdido(medicion)
        if modo == MODE_PREDICTED:
            transcurrido = now - self._instante_ultimo
            confianza = self._ultimo.confidence * (1.0 - transcurrido / self._lost_timeout_s)
            valores = self._ultimo
        else:
            confianza, valores = medicion.confidence, medicion
        return self._estado(modo, (valores.lateral_error_m, valores.heading_error_rad,
                                   valores.curvature), confianza, medicion)

    def _con_filtro(self, modo, medicion: LaneResult, now: float, velocidad_mps: float):
        filtro = self._kalman
        if self._instante_previo is not None:
            filtro.predecir(now - self._instante_previo, velocidad_mps)
        self._instante_previo = now

        if modo in (MODE_TWO_LINES, MODE_ONE_LINE):
            z = (medicion.lateral_error_m, medicion.heading_error_rad, medicion.curvature)
            dos_lineas = modo == MODE_TWO_LINES
            volvio = self._modo_previo == MODE_LOST and dos_lineas
            if not filtro.inicializado or volvio:
                self._reiniciar(z, dos_lineas)
            elif filtro.corregir(z, dos_lineas):
                self._rechazando_desde = None
            elif self._rechazando_desde is None:
                self._rechazando_desde = now
            elif now - self._rechazando_desde > self._lost_timeout_s:
                self._reiniciar(z, dos_lineas)

        if modo == MODE_LOST or not filtro.inicializado:
            return self._perdido(medicion)
        confianza = max(0.0, 1.0 - filtro.desvio_std_m / self._max_std_m)
        return self._estado(modo, filtro.estado, confianza, medicion)

    def _reiniciar(self, z, dos_lineas: bool) -> None:
        self._kalman.reiniciar(z, dos_lineas)
        self._rechazando_desde = None

    @staticmethod
    def _perdido(actual: LaneResult) -> TrackedLane:
        return TrackedLane(MODE_LOST, False, math.nan, math.nan, math.nan, 0.0,
                           actual.intersection_detected, False, False)

    @staticmethod
    def _estado(modo, valores: tuple, confianza: float, actual: LaneResult) -> TrackedLane:
        # La intersección y la validez de las líneas son siempre las de la imagen actual.
        desvio, rumbo, curvatura = valores
        return TrackedLane(modo, True, desvio, rumbo, curvatura, confianza,
                           actual.intersection_detected, actual.left_valid,
                           actual.right_valid)
