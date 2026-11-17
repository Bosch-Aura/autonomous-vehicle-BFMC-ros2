"""Lógica pura de bfmc_control: dirección y velocidad sobre el estado del carril.

El giro lo calcula un algoritmo intercambiable (`algorithm`, ADR-0004), cada uno en su
archivo de algoritmos/: el PID sobre "desvío + peso × rumbo" (A2-138, pid.py) y Pure
Pursuit (A2-155, pure_pursuit.py). Esta clase hace lo que es igual para todos, así una
comparación entre algoritmos solo cambia el algoritmo:

- Sin carril (o con poca confianza): sostiene el último giro hold_steer_s y después
  endereza, a lost_speed_factor de la velocidad de crucero.
- Límite físico: si la curva del carril es más cerrada que R_min = L / tan(max_steer_rad)
  (informe, 4.12.1), baja la velocidad a infeasible_curve_speed_factor.
- Forma de la salida: saturación en max_steer_rad, filtro exponencial y límite de
  velocidad de giro.

No importa ROS ni lee el reloj: recibe el instante actual como argumento, así el
comportamiento en el tiempo se verifica sin esperar.

Convenciones (las mismas que bfmc_interfaces): unidades SI, y un error o un
ángulo positivo es hacia la izquierda.
"""
import math

from bfmc_control import algoritmos
from bfmc_control.algoritmos.base import limitar
from bfmc_control.tipos import (  # noqa: F401 (se reexportan: los usan el nodo y los tests)
    ALGORITMOS, ControlConfig, ControlOutput, LaneInput, PID, PURE_PURSUIT,
)


class ControlLogic:
    """Convierte el estado del carril en un ángulo de dirección y una velocidad."""

    def __init__(self, config: ControlConfig):
        self._config = config
        self._algoritmo = algoritmos.crear(config.algorithm)
        self._last_steer = None
        self._last_time = None
        self._ultimo_giro_usable = 0.0
        self._ultimo_instante_usable = None

    @property
    def config(self) -> ControlConfig:
        """Configuración en uso."""
        return self._config

    @property
    def algoritmo(self):
        """El algoritmo de giro en uso (una instancia de algoritmos.AlgoritmoDeGiro)."""
        return self._algoritmo

    def set_config(self, config: ControlConfig) -> None:
        """Cambia la configuración sin reiniciar el filtro de la salida.

        Si cambia el algoritmo, el nuevo arranca limpio (sin integral ni curvatura vieja).
        Si no, sigue con su estado: cambiar una ganancia en vivo no reinicia el PID.
        """
        if config.algorithm != self._config.algorithm:
            self._algoritmo = algoritmos.crear(config.algorithm)
        self._config = config

    def update(self, lane: LaneInput, now: float) -> ControlOutput:
        """Calcula el comando para el estado del carril recibido.

        Args:
            lane: estado del carril.
            now: instante actual en segundos. Lo provee quien llama.
        """
        cfg = self._config
        usable = lane.detected and lane.confidence >= cfg.min_confidence

        if not usable:
            self._algoritmo.al_perder_el_carril()
            steer = self._dar_forma(self._giro_sin_carril(now), now)
            return ControlOutput(steer, cfg.cruise_speed_mps * cfg.lost_speed_factor)

        objetivo, velocidad = self._algoritmo.calcular(cfg, lane, now)
        steer = self._dar_forma(limitar(objetivo, cfg.max_steer_rad), now)
        if cfg.algorithm == PID:
            # El PID baja la velocidad según cuánto gira (slowdown_gain), como siempre.
            velocidad = cfg.cruise_speed_mps * (
                1.0 - cfg.slowdown_gain * abs(steer) / cfg.max_steer_rad)
        velocidad *= self._factor_curva_imposible(lane)
        self._ultimo_giro_usable = steer
        self._ultimo_instante_usable = now
        return ControlOutput(steer, velocidad)

    def curvatura_maxima(self) -> float:
        """Curvatura más cerrada que el auto puede seguir: tan(max_steer_rad) / L [1/m]."""
        cfg = self._config
        if cfg.pure_pursuit is None:
            return math.inf
        return math.tan(cfg.max_steer_rad) / cfg.pure_pursuit.wheelbase_m

    def _factor_curva_imposible(self, lane: LaneInput) -> float:
        cfg = self._config
        if cfg.infeasible_curve_speed_factor >= 1.0:
            return 1.0
        # Solo con la curvatura confiable: con una línea es ruido y frenaría en las rectas.
        if (lane.confidence >= cfg.curvature_ff_min_confidence and math.isfinite(lane.curvature)
                and abs(lane.curvature) > self.curvatura_maxima()):
            return cfg.infeasible_curve_speed_factor
        return 1.0

    def _giro_sin_carril(self, now):
        """Giro objetivo sin carril: el último usable durante hold_steer_s, después el centro.

        Perder el carril en plena curva suele ser porque las líneas salieron de la vista: si
        se endereza en ese momento el auto se va de la pista. Sostener el giro un rato le da
        tiempo a volver a ver las líneas.
        """
        ultimo = self._ultimo_instante_usable
        if ultimo is not None and 0.0 <= now - ultimo <= self._config.hold_steer_s:
            return self._ultimo_giro_usable
        return 0.0

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
                filtrado = self._last_steer + limitar(filtrado - self._last_steer, maximo)
            steer = filtrado
        steer = limitar(steer, cfg.max_steer_rad)
        self._last_steer = steer
        self._last_time = now
        return steer
