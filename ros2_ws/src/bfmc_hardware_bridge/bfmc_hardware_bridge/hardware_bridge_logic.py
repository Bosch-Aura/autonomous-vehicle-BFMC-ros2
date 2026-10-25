"""Lógica pura de bfmc_hardware_bridge: qué trama enviar al ESP32 y qué reporta.

No importa ROS, no toca el puerto serial y no lee el reloj: recibe el instante
actual como argumento.

Regla de seguridad: el puente frena si la seguridad lo indica o si el último
comando es viejo. Los comandos se reenvían en cada ciclo, y eso hace de
heartbeat para el watchdog del ESP32.
"""
from dataclasses import dataclass
import math
from typing import Optional

from bfmc_hardware_bridge.protocol import (
    encode_brake, encode_drive, parse_line, SEQ_MAX, StatusMessage, UltraMessage,
)
from bfmc_hardware_bridge.units import to_esp32_units


@dataclass(frozen=True)
class BridgeConfig:
    """Parámetros de la lógica. Sus valores vienen de config/params.yaml."""

    link_timeout_s: float
    cmd_timeout_s: float
    max_speed_mps: float
    max_steer_rad: float
    ultrasonic_min_range_m: float
    ultrasonic_max_range_m: float

    def __post_init__(self):
        if self.link_timeout_s <= 0.0 or self.cmd_timeout_s <= 0.0:
            raise ValueError('link_timeout_s y cmd_timeout_s deben ser positivos')
        if self.max_speed_mps <= 0.0 or self.max_steer_rad <= 0.0:
            raise ValueError('max_speed_mps y max_steer_rad deben ser positivos')
        if not 0.0 <= self.ultrasonic_min_range_m < self.ultrasonic_max_range_m:
            raise ValueError('el alcance del ultrasónico debe cumplir 0 <= mínimo < máximo')


@dataclass(frozen=True)
class HardwareState:
    """Estado del enlace con el ESP32."""

    link_up: bool
    watchdog_tripped: bool
    firmware_version: str
    last_seq: int


class BridgeLogic:
    """Decide qué trama enviar y traduce lo que reporta el ESP32."""

    def __init__(self, config: BridgeConfig):
        self._config = config
        self._speed_mps = 0.0
        self._steering_rad = 0.0
        self._cmd_time = None
        self._emergency = False
        self._seq = 0
        self._last_rx = None
        self._firmware = ''
        self._watchdog_tripped = False
        self._esp_last_seq = 0

    def on_drive_command(self, speed_mps: float, steering_rad: float, now: float) -> None:
        """Registra el último comando de conducción recibido en /cmd/drive."""
        self._speed_mps = speed_mps
        self._steering_rad = steering_rad
        self._cmd_time = now

    def on_safety(self, emergency: bool) -> None:
        """Registra el estado de emergencia informado por bfmc_safety."""
        self._emergency = emergency

    def on_serial_line(self, line: str, now: float) -> Optional[float]:
        """Procesa una línea que llegó del ESP32.

        Una línea inválida se descarta y **no** cuenta como señal de vida del enlace.

        Returns:
            La distancia del ultrasónico en metros si la línea era una lectura
            (`+inf` cuando el sensor no recibió eco: camino despejado), o None.
        """
        mensaje = parse_line(line)
        if mensaje is None:
            return None
        self._last_rx = now

        if isinstance(mensaje, StatusMessage):
            self._firmware = mensaje.firmware
            self._watchdog_tripped = mensaje.watchdog_tripped
            self._esp_last_seq = mensaje.last_seq
            return None

        return self._a_distancia(mensaje)

    def next_frame(self, now: float) -> bytes:
        """Devuelve la trama que hay que enviar en este ciclo."""
        seq = self._seq
        self._seq = 0 if self._seq >= SEQ_MAX else self._seq + 1

        comando_vigente = (self._cmd_time is not None
                           and (now - self._cmd_time) <= self._config.cmd_timeout_s)
        if self._emergency or not comando_vigente:
            return encode_brake(seq)

        v_mm_s, s_decimas = to_esp32_units(
            self._speed_mps, self._steering_rad,
            self._config.max_speed_mps, self._config.max_steer_rad)
        return encode_drive(seq, v_mm_s, s_decimas)

    def state(self, now: float) -> HardwareState:
        """Estado del enlace en el instante `now`."""
        link_up = (self._last_rx is not None
                   and (now - self._last_rx) <= self._config.link_timeout_s)
        return HardwareState(
            link_up=link_up,
            watchdog_tripped=self._watchdog_tripped,
            firmware_version=self._firmware,
            last_seq=self._esp_last_seq,
        )

    def _a_distancia(self, mensaje: UltraMessage) -> float:
        if not mensaje.valid:
            return math.inf
        metros = mensaje.dist_cm / 100.0
        cfg = self._config
        return max(cfg.ultrasonic_min_range_m, min(cfg.ultrasonic_max_range_m, metros))
