"""Tipos de bfmc_control: lo que entra, lo que sale y la configuración.

Viven aparte para que la lógica común (control_logic.py) y los algoritmos de giro
(algoritmos/) los compartan sin importarse entre sí. Convenciones (las de
bfmc_interfaces): unidades SI, y un error o un ángulo positivo es hacia la izquierda.
"""
from dataclasses import dataclass
from typing import Optional

from bfmc_control.pure_pursuit import PurePursuitConfig


PID = 'pid'
PURE_PURSUIT = 'pure_pursuit'
# Nombres de los algoritmos de giro. Tienen que coincidir con algoritmos.REGISTRO (lo
# verifica un test): están acá para validar la configuración sin importar los algoritmos.
ALGORITMOS = (PID, PURE_PURSUIT)


@dataclass(frozen=True)
class LaneInput:
    """Lo que la lógica necesita del estado del carril."""

    detected: bool
    lateral_error_m: float
    heading_error_rad: float
    confidence: float
    curvature: float = 0.0               # [1/m], positiva a la izquierda (lo usa Pure Pursuit)
    reference_distance_m: float = 0.0    # del pie de la cámara al punto donde se midió


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
    hold_steer_s: float = 0.0            # [s] giro sostenido al perder el carril (0: al centro ya)
    # Término de curvatura del PID: k · atan(wheelbase · curvatura). 0 = sin término.
    curvature_ff_gain: float = 0.0
    curvature_ff_min_confidence: float = 1.0  # confianza mínima para tomar la curvatura medida
    curvature_ff_max: float = 1.0        # [1/m] tope de la curvatura que se usa
    curvature_ff_hold_s: float = 0.0     # [s] cuánto se sigue usando la última curvatura confiable
    # Espera antes de doblar [m]: la curvatura se mide delante del auto y el término de
    # curvatura dobla apenas la ve. Con turn_delay_m > 0 ese término se retrasa
    # turn_delay_m / v, el tiempo que el auto tarda en avanzar esa distancia. El giro en la
    # curva ya establecida no cambia, solo cuándo empieza. 0 = como antes.
    turn_delay_m: float = 0.0
    # Además, evalúa el desvío y el rumbo esta distancia más cerca del auto sobre la misma
    # línea. Cambia también el giro de régimen en la curva (queda menor). 0 = como antes.
    error_shift_m: float = 0.0
    # Límite físico (informe, 4.12.1): con el tope de dirección max_steer_rad el auto no puede
    # seguir una curva más cerrada que R_min = L / tan(max_steer_rad). Si el carril pide más,
    # la velocidad baja a esta fracción: no hay controlador que la tome, así que se entra más
    # despacio para que el error sea menor. 1 = apagado.
    infeasible_curve_speed_factor: float = 1.0
    algorithm: str = PID
    pure_pursuit: Optional[PurePursuitConfig] = None

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
        if self.hold_steer_s < 0.0:
            raise ValueError('hold_steer_s no puede ser negativo')
        if self.curvature_ff_gain < 0.0 or self.curvature_ff_hold_s < 0.0:
            raise ValueError('curvature_ff_gain y curvature_ff_hold_s no pueden ser negativos')
        if not 0.0 <= self.curvature_ff_min_confidence <= 1.0:
            raise ValueError('curvature_ff_min_confidence debe estar en [0, 1]')
        if self.curvature_ff_max <= 0.0:
            raise ValueError('curvature_ff_max debe ser positivo')
        if not 0.0 <= self.turn_delay_m <= 0.5 or not 0.0 <= self.error_shift_m <= 0.5:
            raise ValueError('turn_delay_m y error_shift_m deben estar en [0, 0,5] m')
        if not 0.0 < self.infeasible_curve_speed_factor <= 1.0:
            raise ValueError('infeasible_curve_speed_factor debe estar en (0, 1]')
        if self.algorithm not in ALGORITMOS:
            raise ValueError('algorithm debe ser uno de: %s' % ', '.join(ALGORITMOS))
        if self.algorithm == PURE_PURSUIT and self.pure_pursuit is None:
            raise ValueError('pure_pursuit necesita sus parámetros')
        if self.curvature_ff_gain > 0.0 and self.pure_pursuit is None:
            raise ValueError('el término de curvatura necesita wheelbase_m (parámetros de '
                             'pure_pursuit)')
        if self.infeasible_curve_speed_factor < 1.0 and self.pure_pursuit is None:
            raise ValueError('el límite de curvatura necesita wheelbase_m (parámetros de '
                             'pure_pursuit)')
