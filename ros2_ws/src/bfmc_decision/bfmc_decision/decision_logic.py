"""Lógica pura de bfmc_decision: la máquina de estados que decide qué hace el auto.

No importa ROS ni lee el reloj: recibe el instante actual como argumento, así el
comportamiento en el tiempo se verifica sin esperar.

Cada ciclo (`step`) elige UNA sola fuente para el comando final, de mayor a menor
prioridad:

1. Emergencia: un obstáculo que frenó al ESP32, el enlace con el hardware o entradas
   vencidas. Se detiene.
2. Maniobra activa: se usa el comando de la maniobra y se ignora el carril.
3. Evento de señal o intersección: se pide una maniobra.
4. Seguir carril: se reenvía el comando del control. Es lo normal.

Convenciones: unidades SI, y un ángulo positivo es hacia la izquierda.
"""
from dataclasses import dataclass
import enum
import math
from typing import Optional, Sequence


class State(enum.IntEnum):
    """Estado de la máquina. Los valores coinciden con VehicleState.STATE_*."""

    STARTING = 0
    LANE_FOLLOW = 1
    MANEUVER = 2
    EMERGENCY = 3


class Source(enum.Enum):
    """De dónde sale el comando que se publica."""

    NONE = 'none'
    LANE = 'lane'
    MANEUVER = 'maneuver'


@dataclass(frozen=True)
class Drive:
    """Comando de conducción en unidades SI."""

    speed_mps: float
    steering_rad: float


STOPPED = Drive(0.0, 0.0)


@dataclass(frozen=True)
class LaneObservation:

    detected: bool
    intersection_detected: bool


@dataclass(frozen=True)
class SignObservation:

    label: str
    confidence: float
    distance_m: float


@dataclass(frozen=True)
class HardwareObservation:
    """Lo que informa el puente del ESP32: enlace, watchdog y si se detuvo por un obstáculo."""

    link_up: bool
    watchdog_tripped: bool
    obstacle_stop: bool = False
    obstacle_distance_m: float = math.nan


@dataclass(frozen=True)
class ManeuverRequest:
    """Una maniobra que hay que pedirle a bfmc_maneuvers."""

    name: str
    direction: str


@dataclass(frozen=True)
class Decision:
    """Resultado de un ciclo: el comando y el estado, y qué hacer con las maniobras."""

    command: Drive
    source: Source
    state: State
    reason: str
    active_maneuver: str
    maneuver_request: Optional[ManeuverRequest]
    cancel_maneuver: bool


@dataclass(frozen=True)
class DecisionConfig:
    """Parámetros de la lógica. Sus valores vienen de config/params.yaml."""

    hardware_timeout_s: float
    lane_cmd_timeout_s: float
    maneuver_cmd_timeout_s: float
    signs_timeout_s: float
    lane_lost_timeout_s: float
    maneuver_request_timeout_s: float
    maneuver_timeout_s: float
    maneuver_cooldown_s: float
    min_sign_confidence: float
    stop_distance_m: float
    intersection_distance_m: float
    stop_labels: Sequence[str]
    intersection_labels: Sequence[str]
    intersection_direction: str
    intersection_min_frames: int

    def __post_init__(self):
        tiempos = (
            self.hardware_timeout_s, self.lane_cmd_timeout_s, self.maneuver_cmd_timeout_s,
            self.signs_timeout_s, self.lane_lost_timeout_s, self.maneuver_request_timeout_s,
            self.maneuver_timeout_s)
        if min(tiempos) <= 0.0:
            raise ValueError('todos los tiempos límite deben ser positivos')
        if self.maneuver_cooldown_s < 0.0:
            raise ValueError('maneuver_cooldown_s no puede ser negativo')
        if not 0.0 <= self.min_sign_confidence <= 1.0:
            raise ValueError('min_sign_confidence debe estar en [0, 1]')
        if self.stop_distance_m <= 0.0 or self.intersection_distance_m <= 0.0:
            raise ValueError('las distancias de activación deben ser positivas')
        if self.intersection_direction not in ('left', 'right', 'straight'):
            raise ValueError('intersection_direction debe ser left, right o straight')
        if self.intersection_min_frames < 1:
            raise ValueError('intersection_min_frames debe ser al menos 1')


class _Maneuver(enum.Enum):

    NONE = 0
    PENDING = 1   # pedida, todavía sin respuesta del servidor
    ACTIVE = 2    # aceptada, en ejecución


class DecisionLogic:
    """Arbitra entre emergencia, maniobra, eventos y seguimiento de carril."""

    def __init__(self, config: DecisionConfig):
        self._config = config

        self._lane_lost_since = None
        self._intersection_streak = 0
        self._signs = ()
        self._signs_time = None
        self._hardware = None
        self._hardware_time = None
        self._lane_cmd = None
        self._lane_cmd_time = None
        self._maneuver_cmd = None
        self._maneuver_cmd_time = None

        self._maneuver = _Maneuver.NONE
        self._maneuver_name = ''
        self._maneuver_since = None
        self._cooldown_until = None

    # --- entradas -----------------------------------------------------------------
    def on_lane(self, lane: LaneObservation, now: float) -> None:
        if lane.detected:
            self._lane_lost_since = None
        elif self._lane_lost_since is None:
            self._lane_lost_since = now
        self._intersection_streak = (
            self._intersection_streak + 1 if lane.intersection_detected else 0)

    def on_signs(self, signs: Sequence[SignObservation], now: float) -> None:
        self._signs = tuple(signs)
        self._signs_time = now

    def on_hardware(self, hardware: HardwareObservation, now: float) -> None:
        self._hardware = hardware
        self._hardware_time = now

    def on_lane_command(self, command: Drive, now: float) -> None:
        self._lane_cmd = command
        self._lane_cmd_time = now

    def on_maneuver_command(self, command: Drive, now: float) -> None:
        self._maneuver_cmd = command
        self._maneuver_cmd_time = now

    def on_maneuver_goal_response(self, accepted: bool, now: float) -> None:
        """El servidor aceptó o rechazó la maniobra pedida."""
        if self._maneuver is not _Maneuver.PENDING:
            return
        if accepted:
            self._maneuver = _Maneuver.ACTIVE
            self._maneuver_since = now
        else:
            self._terminar_maniobra(now)

    def on_maneuver_result(self, now: float) -> None:
        """La maniobra terminó (con éxito, cancelada o con error)."""
        if self._maneuver is _Maneuver.ACTIVE:
            self._terminar_maniobra(now)

    # --- decisión -------------------------------------------------------------------
    def step(self, now: float) -> Decision:
        """Elige el comando y el estado para el instante `now`."""
        if self._faltan_primeras_entradas():
            return self._decision(STOPPED, Source.NONE, State.STARTING,
                                  'esperando las primeras entradas')

        motivo = self._motivo_de_emergencia(now)
        if motivo is not None:
            return self._emergencia(motivo, now)

        if self._maneuver is not _Maneuver.NONE:
            return self._paso_de_maniobra(now)

        pedido = self._evento(now)
        if pedido is not None:
            self._maneuver = _Maneuver.PENDING
            self._maneuver_name = pedido.name
            self._maneuver_since = now
            return self._decision(STOPPED, Source.NONE, State.MANEUVER,
                                  'maniobra pedida: %s' % pedido.name,
                                  maneuver=pedido.name, request=pedido)

        return self._decision(self._lane_cmd, Source.LANE, State.LANE_FOLLOW,
                              'siguiendo el carril')

    # --- emergencia -----------------------------------------------------------------
    def _faltan_primeras_entradas(self):
        return self._hardware_time is None or self._lane_cmd_time is None

    def _motivo_de_emergencia(self, now):
        """Devuelve el motivo de la emergencia, o None si no hay."""
        cfg = self._config

        # Estas condiciones valen siempre, también durante una maniobra.
        if (now - self._hardware_time) > cfg.hardware_timeout_s:
            return 'estado del hardware vencido'
        if not self._hardware.link_up:
            return 'enlace con el ESP32 caído'
        if self._hardware.watchdog_tripped:
            return 'el watchdog del ESP32 cortó los motores'
        if self._hardware.obstacle_stop:
            # El que frenó es el firmware; acá se refleja y se cancela la maniobra en curso.
            distancia = self._hardware.obstacle_distance_m
            if math.isnan(distancia):
                return 'obstáculo: el ESP32 se detuvo'
            return 'obstáculo a %.2f m: el ESP32 se detuvo' % distancia

        # Las que dependen del carril NO valen durante una maniobra: dentro de una
        # intersección las líneas desaparecen a propósito y la maniobra no las usa.
        if self._maneuver is _Maneuver.NONE:
            if (now - self._lane_cmd_time) > cfg.lane_cmd_timeout_s:
                return 'comando de carril vencido'
            if (self._lane_lost_since is not None
                    and (now - self._lane_lost_since) > cfg.lane_lost_timeout_s):
                return 'carril perdido'
        return None

    def _emergencia(self, motivo, now):
        cancelar = self._cancelar_maniobra_en_curso(now)
        return self._decision(STOPPED, Source.NONE, State.EMERGENCY, motivo, cancel=cancelar)

    # --- maniobras --------------------------------------------------------------------
    def _paso_de_maniobra(self, now):
        cfg = self._config
        nombre = self._maneuver_name
        espera = now - self._maneuver_since

        if self._maneuver is _Maneuver.PENDING:
            if espera > cfg.maneuver_request_timeout_s:
                return self._emergencia('el servidor de maniobras no respondió', now)
            return self._decision(STOPPED, Source.NONE, State.MANEUVER,
                                  'esperando al servidor de maniobras', maneuver=nombre)

        if espera > cfg.maneuver_timeout_s:
            return self._emergencia('la maniobra superó su tiempo máximo', now)

        fresco = (self._maneuver_cmd_time is not None
                  and (now - self._maneuver_cmd_time) <= cfg.maneuver_cmd_timeout_s)
        if not fresco:
            return self._decision(STOPPED, Source.NONE, State.MANEUVER,
                                  'maniobra sin comando vigente', maneuver=nombre)
        return self._decision(self._maneuver_cmd, Source.MANEUVER, State.MANEUVER,
                              'ejecutando %s' % nombre, maneuver=nombre)

    def _cancelar_maniobra_en_curso(self, now):
        """Descarta la maniobra. Devuelve True si había una que cancelar."""
        if self._maneuver is _Maneuver.NONE:
            return False
        self._terminar_maniobra(now)
        return True

    def _terminar_maniobra(self, now):
        self._maneuver = _Maneuver.NONE
        self._maneuver_name = ''
        self._cooldown_until = now + self._config.maneuver_cooldown_s
        # Lo observado del carril durante la maniobra no vale: se empieza de cero.
        self._lane_lost_since = None
        self._intersection_streak = 0

    # --- eventos ----------------------------------------------------------------------
    def _evento(self, now):
        if self._cooldown_until is not None and now < self._cooldown_until:
            return None
        cfg = self._config

        # El stop tiene prioridad sobre la intersección.
        if self._hay_senal(cfg.stop_labels, cfg.stop_distance_m, now):
            return ManeuverRequest('stop', '')
        if (self._hay_senal(cfg.intersection_labels, cfg.intersection_distance_m, now)
                or self._intersection_streak >= cfg.intersection_min_frames):
            return ManeuverRequest('intersection', cfg.intersection_direction)
        return None

    def _hay_senal(self, etiquetas, distancia_maxima, now):
        # Si el detector dejó de publicar, sus últimas detecciones ya no valen.
        if self._signs_time is None or (now - self._signs_time) > self._config.signs_timeout_s:
            return False
        for senal in self._signs:
            if senal.label not in etiquetas:
                continue
            if senal.confidence < self._config.min_sign_confidence:
                continue
            # Sin distancia conocida no se puede juzgar si la señal está cerca: se ignora.
            if math.isnan(senal.distance_m) or senal.distance_m > distancia_maxima:
                continue
            return True
        return False

    # --- salida -------------------------------------------------------------------------
    @staticmethod
    def _decision(comando, fuente, estado, motivo, maneuver='', request=None, cancel=False):
        return Decision(
            command=comando,
            source=fuente,
            state=estado,
            reason=motivo,
            active_maneuver=maneuver,
            maneuver_request=request,
            cancel_maneuver=cancel,
        )
