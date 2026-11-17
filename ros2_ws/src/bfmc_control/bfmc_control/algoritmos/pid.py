"""PID sobre "desvío + peso × rumbo" con término de curvatura (A2-138).

Es la arquitectura híbrida del informe (4.7.5 y 5.2.4.6): un módulo geométrico junta la
posición actual (desvío) y la dirección futura (rumbo) en una sola variable de error, y un
PID la corrige. Encima se suma el giro que pide la curva por sí misma,
k · atan(L · curvatura), que es la parte del giro que el auto necesita para seguir un arco
aunque vaya centrado y alineado (modelo de bicicleta, informe 4.12.1).
"""
import math

from bfmc_control.algoritmos.base import AlgoritmoDeGiro, limitar


class Pid(AlgoritmoDeGiro):
    """PID con término de curvatura, espera antes de doblar y corrimiento del error."""

    def __init__(self):
        super().__init__()
        self._reiniciar_pid()

    def reiniciar(self) -> None:
        super().reiniciar()
        self._reiniciar_pid()

    def al_perder_el_carril(self) -> None:
        """Descarga el integral y el derivativo; la curvatura sostenida queda."""
        self._reiniciar_pid()

    def _reiniciar_pid(self):
        self._integral_rad = 0.0
        self._prev_error = None
        self._prev_time = None

    def calcular(self, cfg, lane, now):
        curvatura = self.curvatura.actualizar(cfg, lane, now)
        desvio, rumbo = self._error_mas_cerca(cfg, lane, curvatura)
        error = desvio + cfg.heading_weight_m_per_rad * rumbo
        if abs(error) < cfg.deadband_m:
            error = 0.0

        dt = None if self._prev_time is None else now - self._prev_time
        if dt is None or dt <= 0.0 or dt > cfg.max_dt_s:
            self._reiniciar_pid()
            dt = None

        derivativo = 0.0
        if dt is not None:
            self._integral_rad = limitar(
                self._integral_rad + cfg.ki * error * dt, cfg.integral_limit_rad)
            derivativo = cfg.kd * (error - self._prev_error) / dt

        giro = limitar(cfg.kp * error + self._integral_rad + derivativo
                       + self._termino_de_curvatura(cfg, now), cfg.max_steer_rad)
        self._prev_error = error
        self._prev_time = now
        # La velocidad del PID depende del giro ya filtrado: la calcula ControlLogic con
        # slowdown_gain. Acá se devuelve la de crucero.
        return giro, cfg.cruise_speed_mps

    @staticmethod
    def _error_mas_cerca(cfg, lane, curvatura):
        """Desvío y rumbo evaluados error_shift_m más cerca del auto, sobre la misma línea.

        El carril se midió en un punto delante del auto: con s = -error_shift_m, el centro del
        carril ahí es y(s) = d + psi * s + curvatura * s^2 / 2, y su rumbo es psi + curvatura * s.
        """
        corrimiento = cfg.error_shift_m
        if corrimiento <= 0.0:
            return lane.lateral_error_m, lane.heading_error_rad
        s = -corrimiento
        desvio = (lane.lateral_error_m + lane.heading_error_rad * s
                  + 0.5 * curvatura * s * s)
        return desvio, lane.heading_error_rad + curvatura * s

    def _termino_de_curvatura(self, cfg, now):
        """Giro que pide la curva por sí misma: k · atan(wheelbase · curvatura).

        La curvatura se mide delante del auto: con turn_delay_m se usa la que se midió hace
        turn_delay_m / v, así el giro de la curva empieza cuando el auto llega a ella y no
        cuando aparece en la imagen.
        """
        if cfg.curvature_ff_gain <= 0.0:
            return 0.0
        curvatura = self._curvatura_con_retraso(cfg, now)
        return cfg.curvature_ff_gain * math.atan(cfg.pure_pursuit.wheelbase_m * curvatura)

    def _curvatura_con_retraso(self, cfg, now):
        """La última curvatura confiable medida hasta hace turn_delay_m / v segundos."""
        retraso_s = cfg.turn_delay_m / max(cfg.cruise_speed_mps, 0.1)
        objetivo = now - retraso_s
        elegida = None
        for instante, curvatura in self.curvatura.muestras:
            if instante > objetivo:
                break
            elegida = (instante, curvatura)
        if elegida is None or objetivo - elegida[0] > cfg.curvature_ff_hold_s:
            return 0.0
        return elegida[1]
