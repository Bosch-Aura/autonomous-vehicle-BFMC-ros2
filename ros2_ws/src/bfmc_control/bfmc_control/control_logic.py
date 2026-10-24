"""Lógica pura de bfmc_control: PID de dirección sobre el estado del carril.

No importa ROS ni lee el reloj: recibe el instante actual como argumento, así el
comportamiento en el tiempo se verifica sin esperar.

Convenciones (las mismas que bfmc_interfaces): unidades SI, y un error o un
ángulo positivo es hacia la izquierda.
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class LaneInput:
    """Lo que la lógica necesita del estado del carril."""

    detected: bool
    lateral_error_m: float
    heading_error_rad: float
    confidence: float


@dataclass(frozen=True)
class ControlOutput:
    """Comando de conducción en unidades SI."""

    steering_rad: float
    speed_mps: float


@dataclass(frozen=True)
class ControlConfig:
    """Parámetros de la lógica. Sus valores vienen de config/params.yaml."""

    kp: float
    ki: float
    kd: float
    heading_weight_m_per_rad: float
    deadband_m: float
    integral_limit_rad: float
    max_steer_rad: float
    max_steer_rate_rad_s: float
    filter_alpha: float
    min_confidence: float
    cruise_speed_mps: float
    lost_speed_factor: float
    slowdown_gain: float
    max_dt_s: float

    def __post_init__(self):
        if min(self.kp, self.ki, self.kd) < 0.0:
            raise ValueError('las ganancias kp, ki y kd no pueden ser negativas')
        if self.heading_weight_m_per_rad < 0.0 or self.deadband_m < 0.0:
            raise ValueError('heading_weight_m_per_rad y deadband_m no pueden ser negativos')
        if self.integral_limit_rad < 0.0:
            raise ValueError('integral_limit_rad no puede ser negativo')
        if self.max_steer_rad <= 0.0:
            raise ValueError('max_steer_rad debe ser positivo')
        if self.max_steer_rate_rad_s <= 0.0:
            raise ValueError('max_steer_rate_rad_s debe ser positivo')
        if not 0.0 < self.filter_alpha <= 1.0:
            raise ValueError('filter_alpha debe estar en (0, 1]')
        if not 0.0 <= self.min_confidence <= 1.0:
            raise ValueError('min_confidence debe estar en [0, 1]')
        if self.cruise_speed_mps < 0.0:
            raise ValueError('cruise_speed_mps no puede ser negativa')
        if not 0.0 <= self.lost_speed_factor <= 1.0:
            raise ValueError('lost_speed_factor debe estar en [0, 1]')
        if not 0.0 <= self.slowdown_gain < 1.0:
            raise ValueError('slowdown_gain debe estar en [0, 1)')
        if self.max_dt_s <= 0.0:
            raise ValueError('max_dt_s debe ser positivo')


def _limitar(valor, maximo):
    return max(-maximo, min(maximo, valor))


class ControlLogic:
    """Convierte el estado del carril en un ángulo de dirección y una velocidad."""

    def __init__(self, config: ControlConfig):
        self._config = config
        self._reset_pid()
        self._last_steer = None
        self._last_time = None

    @property
    def config(self) -> ControlConfig:
        """Configuración en uso."""
        return self._config

    def set_config(self, config: ControlConfig) -> None:
        """Cambia la configuración sin reiniciar el PID ni el filtro de la salida."""
        self._config = config

    def _reset_pid(self):
        self._integral_rad = 0.0
        self._prev_error = None
        self._prev_time = None

    def update(self, lane: LaneInput, now: float) -> ControlOutput:
        """Calcula el comando para el estado del carril recibido.

        Args:
            lane: estado del carril.
            now: instante actual en segundos. Lo provee quien llama.
        """
        cfg = self._config
        usable = lane.detected and lane.confidence >= cfg.min_confidence

        if not usable:
            self._reset_pid()
            steer = self._dar_forma(0.0, now)
            return ControlOutput(steer, cfg.cruise_speed_mps * cfg.lost_speed_factor)

        error = lane.lateral_error_m + cfg.heading_weight_m_per_rad * lane.heading_error_rad
        if abs(error) < cfg.deadband_m:
            error = 0.0

        dt = None if self._prev_time is None else now - self._prev_time
        if dt is None or dt <= 0.0 or dt > cfg.max_dt_s:
            self._reset_pid()
            dt = None

        derivative = 0.0
        if dt is not None:
            self._integral_rad = _limitar(
                self._integral_rad + cfg.ki * error * dt, cfg.integral_limit_rad)
            derivative = cfg.kd * (error - self._prev_error) / dt

        raw = _limitar(cfg.kp * error + self._integral_rad + derivative, cfg.max_steer_rad)
        self._prev_error = error
        self._prev_time = now

        steer = self._dar_forma(raw, now)
        speed = cfg.cruise_speed_mps * (1.0 - cfg.slowdown_gain * abs(steer) / cfg.max_steer_rad)
        return ControlOutput(steer, speed)

    def _dar_forma(self, objetivo, now):
        """Filtra la salida y limita su velocidad de cambio."""
        cfg = self._config
        if self._last_steer is None:
            steer = objetivo
        else:
            filtrado = cfg.filter_alpha * objetivo + (1.0 - cfg.filter_alpha) * self._last_steer
            dt = now - self._last_time
            if dt > 0.0:
                maximo = cfg.max_steer_rate_rad_s * dt
                filtrado = self._last_steer + _limitar(filtrado - self._last_steer, maximo)
            steer = filtrado
        steer = _limitar(steer, cfg.max_steer_rad)
        self._last_steer = steer
        self._last_time = now
        return steer
