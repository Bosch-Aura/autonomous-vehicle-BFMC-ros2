"""Lógica pura de bfmc_hardware_bridge: qué enviar al ESP32 y qué reporta.

No importa ROS, no toca el puerto serial y no lee el reloj: recibe el instante
actual como argumento.

Reglas (ADR-0002 y ADR-0003), para el firmware actual de Bosch-Aura/embedded:

- En cada ciclo se envían dirección y velocidad, aunque no hayan cambiado. Eso
  alimenta el watchdog del ESP32 (300 ms). El firmware deja la velocidad
  latcheada; si el último comando de ROS es viejo, se envía velocidad 0 y
  dirección al centro.
- El firmware solo emite eventos cuando algo cambia: andando a velocidad
  constante no dice nada. Para saber que sigue vivo, el puente reenvía cada
  `heartbeat_period_s` el pedido de stop vigente (M:SYS_STOP o M:SYS_RESUME).
  Como el ESP32 ya está así, no cambia nada, pero siempre lo confirma con
  EVENT:CMD_EXECUTED, y esa respuesta mantiene `link_up`.
- Al abrir el puerto, o si el ESP32 se reinicia, se lo arma. Un FAULT posterior
  (watchdog o E-STOP) **no** se rearma solo: hace falta `request_arm()`.
- Otro nodo puede pedir detener el auto (`request_stop(True)`) o que vuelva a
  arrancar (`request_stop(False)`). Se reintenta hasta que el ESP32 lo confirme.
  El ESP32 olvida el stop al reiniciarse, así que se le vuelve a enviar.
- Ante un obstáculo frena el firmware (bloquea el avance) y avisa con
  OBSTACLE_DETECTED:<cm> y OBSTACLE_CLEARED. El puente lo informa y no manda
  ningún freno por su cuenta.
- Si el obstáculo se va y adelante no queda nada al alcance del ultrasónico, el
  firmware no recibe lecturas lejanas válidas y no suelta el bloqueo. Avisa con
  ULTRASONIC_SILENT a los 2 s sin lecturas. Con `auto_obstacle_reset`, el
  puente manda entonces M:OBSTACLE_RESET una sola vez por frenada: si el ESP32
  vuelve a bloquear enseguida, ya no insiste y hace falta
  `request_obstacle_reset()` (ADR-0003).
- Se envía como mucho un comando de gestión (M:) por ciclo, con esta prioridad:
  stop sin confirmar, armado, reset de obstáculo y latido.
"""
from dataclasses import dataclass
from typing import List, Optional

from bfmc_hardware_bridge.protocol import (
    encode_arm, encode_direction, encode_disarm, encode_obstacle_reset, encode_resume,
    encode_speed, encode_steer, encode_stop, Event, parse_line,
)
from bfmc_hardware_bridge.units import Esp32Limits, to_duty, to_servo

ESTADOS = ('DISARMED', 'RUNNING', 'STOPPED', 'FAULT')
# Armado: el ESP32 obedece o, en STOPPED, está armado pero quieto (obstáculo o
# M:SYS_STOP). En ninguno de estos hace falta armar.
ARMADOS = ('RUNNING', 'STOPPED')


@dataclass(frozen=True)
class BridgeConfig:
    """Parámetros de la lógica. Sus valores vienen de config/params.yaml."""

    link_timeout_s: float
    cmd_timeout_s: float
    arm_retry_s: float
    heartbeat_period_s: float
    auto_obstacle_reset: bool
    obstacle_reblock_window_s: float
    limits: Esp32Limits

    def __post_init__(self):
        tiempos = (self.link_timeout_s, self.cmd_timeout_s, self.arm_retry_s,
                   self.heartbeat_period_s, self.obstacle_reblock_window_s)
        if min(tiempos) <= 0.0:
            raise ValueError('todos los tiempos deben ser positivos')
        if self.heartbeat_period_s >= self.link_timeout_s:
            raise ValueError('heartbeat_period_s debe ser menor que link_timeout_s')


@dataclass(frozen=True)
class HardwareState:
    """Estado del enlace con el ESP32.

    `obstacle` es verdadero mientras el ESP32 bloquea el avance por el
    ultrasónico; `obstacle_cm` es la distancia a la que lo vio, o None si no la
    informó. `ultrasonic_silent` indica que el sensor lleva 2 s sin ninguna
    lectura válida (espacio abierto o sensor desconectado).
    """

    link_up: bool
    esp32_state: str
    obstacle: bool = False
    obstacle_cm: Optional[int] = None
    ultrasonic_silent: bool = False

    @property
    def fault(self) -> bool:
        """Indica si el ESP32 está en FAULT: frenó y no acepta tracción hasta rearmarlo."""
        return self.esp32_state == 'FAULT'

    @property
    def stopped(self) -> bool:
        """Indica si el ESP32 está en STOPPED: armado pero quieto (obstáculo o SYS_STOP)."""
        return self.esp32_state == 'STOPPED'


class BridgeLogic:
    """Decide qué enviar al ESP32 y lleva el estado que este reporta."""

    def __init__(self, config: BridgeConfig):
        self._config = config
        self._speed_mps = 0.0
        self._steering_rad = 0.0
        self._cmd_time = None
        self._last_rx = None
        self._state = ''
        self._obstacle = False
        self._obstacle_cm = None
        self._silent = False
        self._direction = None
        self._want_arm = False
        self._last_arm = None
        # Stop pedido por otro nodo: None si nadie pidió nada todavía.
        self._stop_wanted = None
        self._stop_confirmed = True
        self._last_stop = None
        # Reset del obstáculo: pendiente hasta que el ESP32 lo confirme.
        self._reset_pending = False
        self._last_reset = None
        # Destrabe automático: cuándo se hizo el último y si quedó agotado
        # porque el ESP32 volvió a bloquear enseguida.
        self._auto_reset_time = None
        self._auto_reset_exhausted = False

    # --- entradas -------------------------------------------------------------
    def on_drive_command(self, speed_mps: float, steering_rad: float, now: float) -> None:
        """Registra el último comando de conducción recibido en /cmd/drive."""
        self._speed_mps = speed_mps
        self._steering_rad = steering_rad
        self._cmd_time = now

    def on_port_opened(self) -> None:
        """El puerto se acaba de abrir: no se sabe nada del ESP32, hay que armarlo."""
        self._state = ''
        self._nueva_sesion()
        self._last_rx = None

    def request_arm(self) -> None:
        """Pide armar el ESP32, por ejemplo para salir de FAULT."""
        self._want_arm = True
        self._last_arm = None

    def request_stop(self, stop: bool) -> None:
        """Pide detener el auto (True: M:SYS_STOP) o que vuelva a arrancar (False: M:SYS_RESUME).

        El pedido queda guardado y se reenvía hasta que el ESP32 lo confirme;
        después se sigue reenviando como latido. Volver a arrancar solo suelta
        el stop: el auto se mueve con el próximo comando de /cmd/drive, y si hay
        un obstáculo delante sigue detenido.
        """
        if stop != self._stop_wanted:
            self._stop_wanted = stop
            self._stop_confirmed = False
            self._last_stop = None

    def request_obstacle_reset(self) -> None:
        """Pide soltar a mano el bloqueo por obstáculo (M:OBSTACLE_RESET).

        Si el obstáculo sigue ahí, el ESP32 vuelve a bloquear en unos 150 ms.
        También rehabilita el destrabe automático si se había agotado.
        """
        self._reset_pending = True
        self._last_reset = None
        self._auto_reset_exhausted = False
        self._auto_reset_time = None

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
        self._aplicar(evento, now)
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
        if comando_vigente:
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
        return HardwareState(link_up=link_up, esp32_state=self._state,
                             obstacle=self._obstacle, obstacle_cm=self._obstacle_cm,
                             ultrasonic_silent=self._silent)

    def stop_requested(self) -> Optional[bool]:
        """Último pedido de stop: True detener, False arrancar, None si nadie pidió nada."""
        return self._stop_wanted

    # --- internos -------------------------------------------------------------------
    def _nueva_sesion(self):
        """El ESP32 arrancó de cero: DISARMED, sin stop ni obstáculo, hacia adelante.

        Si se pidió detener, hay que volver a enviarlo antes de armar. Un pedido
        de arrancar ya está cumplido.
        """
        self._stop_confirmed = self._stop_wanted is not True
        self._last_stop = None
        self._direction = None
        self._obstacle = False
        self._obstacle_cm = None
        self._silent = False
        self._reset_pending = False
        self._last_reset = None
        self._auto_reset_time = None
        self._auto_reset_exhausted = False
        self._want_arm = self._state not in ARMADOS
        self._last_arm = None

    def _encode_stop_vigente(self):
        # Sin pedido todavía, el ESP32 no tiene ningún stop: el latido lo confirma con RESUME.
        return encode_stop() if self._stop_wanted else encode_resume()

    def _gestion(self, now):
        """Como mucho un comando de gestión por ciclo, por orden de prioridad."""
        cfg = self._config

        # 1. Un pedido de stop o de arranque sin confirmar.
        if not self._stop_confirmed and (
                self._last_stop is None or (now - self._last_stop) >= cfg.arm_retry_s):
            self._last_stop = now
            return self._encode_stop_vigente()

        # 2. Armar.
        if self._want_arm and (
                self._last_arm is None or (now - self._last_arm) >= cfg.arm_retry_s):
            self._last_arm = now
            return encode_arm()

        # 3. Soltar el obstáculo (a mano o automático).
        if self._debe_destrabar_solo():
            self._reset_pending = True
            self._last_reset = None
            self._auto_reset_time = now
        if self._reset_pending and (
                self._last_reset is None or (now - self._last_reset) >= cfg.arm_retry_s):
            self._last_reset = now
            return encode_obstacle_reset()

        # 4. Latido: el mismo pedido de stop, que el ESP32 siempre confirma.
        if self._stop_confirmed and (
                self._last_stop is None or (now - self._last_stop) >= cfg.heartbeat_period_s):
            self._last_stop = now
            return self._encode_stop_vigente()
        return None

    def _debe_destrabar_solo(self):
        return (self._config.auto_obstacle_reset and self._obstacle and self._silent
                and not self._reset_pending and not self._auto_reset_exhausted
                and self._auto_reset_time is None)

    def _aplicar(self, evento: Event, now: float):
        nombre, args = evento.name, evento.args
        if nombre == 'SYSTEM_READY':
            # El ESP32 se reinició. El estado inicial ya llegó: el firmware lo
            # emite antes de este evento.
            self._nueva_sesion()
        elif nombre == 'STATE_CHANGED' and args and args[0] in ESTADOS:
            self._cambiar_estado(args[0])
        elif nombre in ('WATCHDOG_TIMEOUT', 'ESTOP_TRIGGERED'):
            self._cambiar_estado('FAULT')
        elif nombre == 'OBSTACLE_DETECTED':
            self._al_detectar_obstaculo(args, now)
        elif nombre == 'OBSTACLE_CLEARED':
            self._obstacle = False
            self._obstacle_cm = None
            # Mientras está agotado no hay destrabes automáticos: este despeje lo
            # hizo el propio firmware o una persona, y la próxima frenada es otra.
            self._auto_reset_exhausted = False
        elif nombre == 'ULTRASONIC_SILENT':
            self._silent = True
        elif nombre == 'ULTRASONIC_OK':
            self._silent = False
        elif nombre == 'CMD_EXECUTED' and args:
            self._confirmado(args[0])

    def _al_detectar_obstaculo(self, args, now):
        self._obstacle = True
        self._obstacle_cm = int(args[0]) if args and args[0].isdigit() else None
        # Detectar un obstáculo exige lecturas válidas: el sensor no está en silencio.
        self._silent = False
        if self._auto_reset_time is not None:
            # Volvió a bloquear justo después de un destrabe automático: es la
            # misma frenada y no se insiste. Si pasó más tiempo, es otra frenada.
            if (now - self._auto_reset_time) <= self._config.obstacle_reblock_window_s:
                self._auto_reset_exhausted = True
            self._auto_reset_time = None

    def _confirmado(self, comando):
        if comando in ('SYS_STOP', 'SYS_RESUME'):
            # El firmware confirma siempre, aunque el stop ya estuviera así. Una
            # confirmación del pedido anterior no cuenta para el nuevo.
            if (comando == 'SYS_STOP') == bool(self._stop_wanted):
                self._stop_confirmed = True
        elif comando == 'SYS_ARM':
            # Si ya estaba armado, el firmware confirma sin emitir STATE_CHANGED.
            self._want_arm = False
        elif comando == 'OBSTACLE_RESET':
            self._reset_pending = False

    def _cambiar_estado(self, estado):
        self._state = estado
        # Armado conseguido: a partir de acá un FAULT no se rearma solo.
        if estado in ARMADOS:
            self._want_arm = False
