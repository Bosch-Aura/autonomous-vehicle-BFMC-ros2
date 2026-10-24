"""Lógica pura de bfmc_hardware_bridge: qué enviar al ESP32 y qué reporta.

No importa ROS, no toca el puerto serial y no lee el reloj: recibe el instante
actual como argumento.

Reglas (ADR-0002):

- En cada ciclo se envían velocidad y dirección, aunque no hayan cambiado. Eso
  renueva el TTL de 200 ms de los comandos y hace de heartbeat para el watchdog
  del ESP32 (300 ms).
- Si el último comando es viejo, se envía velocidad 0 y dirección al centro.
- Si `bfmc_safety` indica emergencia, se envía un freno de emergencia al empezar
  y velocidad 0 mientras dure.
- Al abrir el puerto, o si el ESP32 se reinicia, se lo arma. El ESP32 no tiene
  modos de manejo: si el auto va solo o lo maneja una persona se decide acá.
  Un FAULT posterior (watchdog, E-STOP u obstáculo) **no** se rearma solo: hace
  falta pedirlo con `request_arm()`.
- Se envía como mucho un comando de gestión por ciclo: el
  firmware guarda uno solo y un segundo, llegado antes de que lo procese,
  pisaría al primero.
"""
from dataclasses import dataclass
from typing import List, Optional

from bfmc_hardware_bridge.protocol import (
    encode_arm, encode_brake_now, encode_direction, encode_disarm, encode_speed,
    encode_steer, Event, parse_line,
)
from bfmc_hardware_bridge.units import Esp32Limits, to_duty, to_servo

ESTADOS = ('DISARMED', 'ARMED', 'RUNNING', 'FAULT')


@dataclass(frozen=True)
class BridgeConfig:
    """Parámetros de la lógica. Sus valores vienen de config/params.yaml."""

    link_timeout_s: float
    cmd_timeout_s: float
    arm_retry_s: float
    limits: Esp32Limits

    def __post_init__(self):
        if min(self.link_timeout_s, self.cmd_timeout_s, self.arm_retry_s) <= 0.0:
            raise ValueError('link_timeout_s, cmd_timeout_s y arm_retry_s deben ser positivos')


@dataclass(frozen=True)
class HardwareState:
    """Estado del enlace con el ESP32."""

    link_up: bool
    esp32_state: str

    @property
    def fault(self) -> bool:
        """Indica si el ESP32 está en FAULT: frenó y no acepta tracción hasta rearmarlo."""
        return self.esp32_state == 'FAULT'


class BridgeLogic:
    """Decide qué enviar al ESP32 y lleva el estado que este reporta."""

    def __init__(self, config: BridgeConfig):
        self._config = config
        self._speed_mps = 0.0
        self._steering_rad = 0.0
        self._cmd_time = None
        self._emergency = False
        self._emergency_sent = False
        self._last_rx = None
        self._state = ''
        self._direction = None
        self._want_arm = False
        self._last_arm = None

    # --- entradas -------------------------------------------------------------
    def on_drive_command(self, speed_mps: float, steering_rad: float, now: float) -> None:
        """Registra el último comando de conducción recibido en /cmd/drive."""
        self._speed_mps = speed_mps
        self._steering_rad = steering_rad
        self._cmd_time = now

    def on_safety(self, emergency: bool) -> None:
        """Registra el estado de emergencia informado por bfmc_safety."""
        if not emergency:
            self._emergency_sent = False
        self._emergency = emergency

    def on_port_opened(self) -> None:
        """El puerto se acaba de abrir: no se sabe nada del ESP32, hay que armarlo."""
        self._olvidar_esp32()
        self._last_rx = None

    def request_arm(self) -> None:
        """Pide armar el ESP32, por ejemplo para salir de FAULT."""
        self._want_arm = True
        self._last_arm = None

    def on_serial_line(self, line: str, now: float) -> Optional[Event]:
        """Procesa una línea que llegó del ESP32.

        Solo los eventos cuentan como señal de vida del enlace; la depuración y
        el ruido se descartan.

        Returns:
            El evento interpretado, o None si la línea no era un evento.
        """
        evento = parse_line(line)
        if evento is None:
            return None
        self._last_rx = now
        self._aplicar(evento)
        return evento

    # --- salida -------------------------------------------------------------------
    def next_frames(self, now: float) -> bytes:
        """Devuelve las líneas que hay que enviar en este ciclo, ya concatenadas."""
        tramas: List[bytes] = []
        gestion = self._gestion(now)
        if gestion is not None:
            tramas.append(gestion)

        limits = self._config.limits
        comando_vigente = (self._cmd_time is not None
                           and (now - self._cmd_time) <= self._config.cmd_timeout_s)
        if self._emergency:
            if not self._emergency_sent:
                tramas.append(encode_brake_now())
                self._emergency_sent = True
            sentido, duty, servo = None, 0, limits.servo_center
        elif comando_vigente:
            sentido, duty = to_duty(self._speed_mps, limits)
            servo = to_servo(self._steering_rad, limits)
        else:
            sentido, duty, servo = None, 0, limits.servo_center

        # El sentido va antes que la velocidad, y solo cuando cambia: el firmware
        # lo recuerda y hace la inversión con su propio tiempo muerto.
        if sentido is not None and sentido != self._direction:
            tramas.append(encode_direction(sentido))
            self._direction = sentido
        tramas.append(encode_steer(servo))
        tramas.append(encode_speed(duty))
        return b''.join(tramas)

    def shutdown_frames(self) -> bytes:
        """Líneas para dejar el auto quieto y desarmado al cerrar el nodo."""
        return encode_speed(0) + encode_disarm()

    def state(self, now: float) -> HardwareState:
        """Estado del enlace en el instante `now`."""
        link_up = (self._last_rx is not None
                   and (now - self._last_rx) <= self._config.link_timeout_s)
        return HardwareState(link_up=link_up, esp32_state=self._state)

    # --- internos -------------------------------------------------------------------
    def _olvidar_esp32(self):
        self._state = ''
        self._nueva_sesion()

    def _nueva_sesion(self):
        """Hay que volver a armar; el sentido recordado ya no vale."""
        self._direction = None
        self._emergency_sent = False
        self._want_arm = self._state not in ('ARMED', 'RUNNING')
        self._last_arm = None

    def _gestion(self, now):
        """Como mucho un comando de gestión por ciclo: el armado, mientras haga falta."""
        if not self._want_arm:
            return None
        if self._last_arm is not None and (now - self._last_arm) < self._config.arm_retry_s:
            return None
        self._last_arm = now
        return encode_arm()

    def _aplicar(self, evento: Event):
        nombre, args = evento.name, evento.args
        if nombre == 'SYSTEM_READY':
            # El ESP32 se reinició y arrancó en su sentido por defecto. El estado
            # ya llegó: el firmware lo emite antes de este evento.
            self._nueva_sesion()
        elif nombre == 'STATE_CHANGED' and args and args[0] in ESTADOS:
            self._cambiar_estado(args[0])
        elif nombre == 'STATE_AUTO_TRANSITION':
            self._cambiar_estado('RUNNING')
        elif nombre in ('WATCHDOG_TIMEOUT', 'FAULT_TRIGGERED', 'ESTOP_TRIGGERED'):
            self._state = 'FAULT'
        elif nombre == 'CMD_EXECUTED' and args and args[0] == 'SYS_ARM':
            # Si ya estaba armado, el firmware confirma sin emitir STATE_CHANGED.
            if self._state != 'RUNNING':
                self._cambiar_estado('ARMED')

    def _cambiar_estado(self, estado):
        self._state = estado
        # Armado conseguido: a partir de acá un FAULT no se rearma solo.
        if estado in ('ARMED', 'RUNNING'):
            self._want_arm = False
