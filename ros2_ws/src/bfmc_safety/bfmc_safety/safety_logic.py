"""Lógica pura de bfmc_safety: decide si el auto debe detenerse según el ultrasónico.

No importa ROS ni lee el reloj: recibe el instante actual como argumento.

Cómo se interpreta una lectura (convención REP-117 de sensor_msgs/Range):

- Distancia finita dentro del alcance del sensor: es la distancia al obstáculo.
- Más allá del alcance, o +inf: no se detectó nada, el camino está despejado.
  El ultrasónico sin eco cae acá: no es un fallo, es un tramo libre.
- Más cerca del mínimo, o -inf: hay un obstáculo tan cerca que el sensor no puede
  medirlo. Se toma como un obstáculo a la distancia mínima.
- NaN: la lectura es un error del sensor.
"""
from dataclasses import dataclass
import enum
import math


class Reason(enum.IntEnum):
    """Motivo de la emergencia. Los valores coinciden con SafetyStatus.REASON_*."""

    NONE = 0
    OBSTACLE = 1
    SENSOR_INVALID = 2
    SENSOR_STALE = 3


@dataclass(frozen=True)
class Reading:
    """Una lectura del ultrasónico, como llega en sensor_msgs/Range."""

    range_m: float
    min_range_m: float
    max_range_m: float


@dataclass(frozen=True)
class SafetyResult:
    """Estado de seguridad en un instante."""

    emergency: bool
    reason: Reason
    sensor_valid: bool
    distance_m: float


@dataclass(frozen=True)
class SafetyConfig:
    """Parámetros de la lógica. Sus valores vienen de config/params.yaml."""

    stop_distance_m: float
    resume_distance_m: float
    stale_timeout_s: float
    emergency_on_invalid: bool

    def __post_init__(self):
        if self.stop_distance_m <= 0.0:
            raise ValueError('stop_distance_m debe ser positivo')
        if self.resume_distance_m <= self.stop_distance_m:
            raise ValueError(
                'resume_distance_m (%s) debe ser mayor que stop_distance_m (%s): '
                'si no, no hay histéresis y el estado oscilaría'
                % (self.resume_distance_m, self.stop_distance_m))
        if self.stale_timeout_s <= 0.0:
            raise ValueError('stale_timeout_s debe ser positivo')


class SafetyLogic:
    """Combina las lecturas del ultrasónico en un estado de seguridad."""

    def __init__(self, config: SafetyConfig):
        self._config = config
        self._last_reading = None
        self._last_time = None
        self._obstacle_latched = False

    def on_reading(self, reading: Reading, now: float) -> None:
        """Registra una lectura nueva recibida en el instante `now` (segundos)."""
        self._last_reading = reading
        self._last_time = now

    def evaluate(self, now: float) -> SafetyResult:
        """Calcula el estado de seguridad en el instante `now`."""
        cfg = self._config

        # Sin lecturas, o con la última demasiado vieja: no se puede saber qué hay adelante.
        if self._last_time is None or (now - self._last_time) > cfg.stale_timeout_s:
            self._obstacle_latched = False
            return SafetyResult(True, Reason.SENSOR_STALE, False, math.nan)

        reading = self._last_reading
        if math.isnan(reading.range_m):
            self._obstacle_latched = False
            if cfg.emergency_on_invalid:
                return SafetyResult(True, Reason.SENSOR_INVALID, False, math.nan)
            return SafetyResult(False, Reason.NONE, False, math.nan)

        distance = self._distancia(reading)

        # Histéresis: se detiene por debajo de stop_distance_m y solo se libera por
        # encima de resume_distance_m.
        if distance < cfg.stop_distance_m:
            self._obstacle_latched = True
        elif distance > cfg.resume_distance_m:
            self._obstacle_latched = False

        reason = Reason.OBSTACLE if self._obstacle_latched else Reason.NONE
        return SafetyResult(self._obstacle_latched, reason, True, distance)

    @staticmethod
    def _distancia(reading: Reading) -> float:
        if reading.range_m < reading.min_range_m:
            return reading.min_range_m
        if reading.range_m > reading.max_range_m:
            return reading.max_range_m
        return reading.range_m
