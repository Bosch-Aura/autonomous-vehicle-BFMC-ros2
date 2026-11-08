"""Pure Pursuit sobre el estado del carril (A2-155). Lógica pura: sin ROS ni reloj.

Marco del auto: origen en el eje trasero, x hacia adelante, y hacia la izquierda, metros.

El estado del carril (desvío d, rumbo psi y curvatura kappa de bfmc_interfaces/LaneState)
NO se mide en el auto sino en un punto delante de la cámara: `reference_distance_m` desde
el pie de la cámara, o sea x_ref = camera_to_rear_axle_m + reference_distance_m desde el
eje trasero. El centro del carril cerca de ese punto es, con s = x - x_ref:

    y(x) = d + psi·s + kappa·s²/2

(d ya es la posición del centro respecto del auto, positiva a la izquierda: no hay que
cambiarle el signo). El punto objetivo es (Ld, y(Ld)) y el giro que lleva al auto en un arco
hasta él es delta = atan(2·L·sin(alpha) / l), con alpha = atan2(y, x) y l la distancia al
punto. L es la distancia entre ejes.

La velocidad baja en las curvas para no pasar `max_lateral_accel_mps2` (v² · |kappa|): es
la práctica común de adaptación de velocidad en curvas. Y baja con la confianza, con un
piso: menos certeza, menos velocidad (la misma degradación gradual de A2-152).
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PurePursuitConfig:
    """Parámetros de Pure Pursuit. Sus valores vienen de config/params.yaml."""

    wheelbase_m: float                 # distancia entre ejes
    camera_to_rear_axle_m: float       # del eje trasero al pie de la cámara, hacia adelante
    lookahead_min_m: float             # Ld mínima, desde el eje trasero
    lookahead_max_m: float             # Ld máxima
    lookahead_gain_s: float            # Ld = gain · v (0: Ld fija en la mínima)
    max_lateral_accel_mps2: float      # aceleración lateral máxima en las curvas
    speed_min_factor: float            # fracción mínima de la velocidad por baja confianza

    def __post_init__(self):
        if self.wheelbase_m <= 0.0 or self.camera_to_rear_axle_m < 0.0:
            raise ValueError('wheelbase_m debe ser positivo y camera_to_rear_axle_m no '
                             'negativo')
        if not 0.0 < self.lookahead_min_m <= self.lookahead_max_m:
            raise ValueError('hace falta 0 < lookahead_min_m <= lookahead_max_m')
        if self.lookahead_gain_s < 0.0 or self.max_lateral_accel_mps2 <= 0.0:
            raise ValueError('lookahead_gain_s no puede ser negativo y '
                             'max_lateral_accel_mps2 debe ser positivo')
        if not 0.0 < self.speed_min_factor <= 1.0:
            raise ValueError('speed_min_factor debe estar en (0, 1]')


def distancia_de_mirada(cfg: PurePursuitConfig, velocidad_mps: float) -> float:
    """Ld = clamp(gain · v, mínima, máxima)."""
    return min(cfg.lookahead_max_m,
               max(cfg.lookahead_min_m, cfg.lookahead_gain_s * abs(velocidad_mps)))


def punto_objetivo(cfg: PurePursuitConfig, lateral_m: float, rumbo_rad: float,
                   curvatura: float, reference_distance_m: float, ld: float) -> tuple:
    """(x, y) del centro del carril a Ld del eje trasero, en el marco del auto."""
    s = ld - (cfg.camera_to_rear_axle_m + reference_distance_m)
    return ld, lateral_m + rumbo_rad * s + 0.5 * curvatura * s * s


def angulo_de_direccion(cfg: PurePursuitConfig, x: float, y: float) -> float:
    """Giro [rad] que lleva al auto en un arco hasta (x, y). Positivo a la izquierda."""
    alpha = math.atan2(y, x)
    return math.atan(2.0 * cfg.wheelbase_m * math.sin(alpha) / math.hypot(x, y))


def velocidad(cfg: PurePursuitConfig, crucero_mps: float, curvatura: float,
              confianza: float) -> float:
    """Velocidad de crucero limitada por la curva y reducida por la confianza."""
    v = crucero_mps
    if abs(curvatura) > 1e-9:
        v = min(v, math.sqrt(cfg.max_lateral_accel_mps2 / abs(curvatura)))
    return v * max(cfg.speed_min_factor, min(1.0, confianza))
