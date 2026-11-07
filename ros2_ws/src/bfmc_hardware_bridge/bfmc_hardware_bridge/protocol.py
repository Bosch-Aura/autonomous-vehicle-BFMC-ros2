r"""Protocolo serial Jetson <-> ESP32 (ADR-0002). Sin ROS y sin puerto serial.

Es el protocolo del firmware de Bosch-Aura/embedded (docs/BRAIN_TEAM_PROTOCOL.md):
una línea de texto ASCII por mensaje, terminada en '\n'.

    Jetson -> ESP32   CANAL:COMANDO:VALOR
                      C:SET_SPEED:<0-255>   duty de tracción, siempre >= 0
                      C:SET_STEER:<50-160>  servo: 50 izquierda, 105 centro, 160 derecha
                      C:SET_DIR:<0|1>       sentido: 1 adelante, 0 atrás
                      E:BRAKE_NOW:0         freno de un solo uso (no lo usa el puente)
                      M:SYS_ARM:0 / M:SYS_DISARM:0
                      M:SYS_STOP:0 / M:SYS_RESUME:0  STOPPED enganchado / soltarlo
                      M:OBSTACLE_RESET:0    suelta el bloqueo por obstáculo

    ESP32 -> Jetson   EVENT:<NOMBRE>[:<arg>...]
                      EVENT:STATE_CHANGED:RUNNING, EVENT:CMD_EXECUTED:SYS_STOP, ...

Los eventos salen solo cuando algo cambia, o como confirmación de un comando M:.
Cualquier otra línea del ESP32 (por ejemplo "[Control] started") es depuración
para humanos y no forma parte del protocolo.
"""
from dataclasses import dataclass
from typing import Optional, Tuple

PREFIJO_EVENTO = 'EVENT:'


@dataclass(frozen=True)
class Event:
    """Un evento del ESP32.

    `EVENT:CMD_EXECUTED:SET_SPEED:150` -> name `CMD_EXECUTED`, args `('SET_SPEED', '150')`.
    """

    name: str
    args: Tuple[str, ...]


def _comando(canal: str, comando: str, valor: int) -> bytes:
    return ('%s:%s:%d\n' % (canal, comando, valor)).encode('ascii')


def encode_speed(duty: int) -> bytes:
    """Duty de tracción. El sentido va aparte, con encode_direction()."""
    return _comando('C', 'SET_SPEED', duty)


def encode_steer(servo: int) -> bytes:
    """Posición del servo de dirección."""
    return _comando('C', 'SET_STEER', servo)


def encode_direction(forward: bool) -> bytes:
    """Sentido de marcha. El firmware lo recuerda hasta que se cambia."""
    return _comando('C', 'SET_DIR', 1 if forward else 0)


def encode_brake_now() -> bytes:
    """Freno de un solo uso: el próximo C:SET_SPEED vuelve a mover el auto."""
    return _comando('E', 'BRAKE_NOW', 0)


def encode_arm() -> bytes:
    """Arma el sistema. También es la forma de salir de FAULT."""
    return _comando('M', 'SYS_ARM', 0)


def encode_disarm() -> bytes:
    """Desarma el sistema: frena y centra la dirección."""
    return _comando('M', 'SYS_DISARM', 0)


def encode_stop() -> bytes:
    """Deja el ESP32 en STOPPED (velocidad 0, servo quieto) hasta un SYS_RESUME."""
    return _comando('M', 'SYS_STOP', 0)


def encode_resume() -> bytes:
    """Suelta el SYS_STOP. Con un obstáculo delante el ESP32 sigue en STOPPED."""
    return _comando('M', 'SYS_RESUME', 0)


def encode_obstacle_reset() -> bytes:
    """Suelta el bloqueo por obstáculo. Si sigue ahí, el ESP32 vuelve a bloquear en ~150 ms."""
    return _comando('M', 'OBSTACLE_RESET', 0)


def parse_line(linea: str) -> Optional[Event]:
    """Interpreta una línea del ESP32.

    Devuelve un `Event`, o None si la línea no es un evento (depuración, ruido o
    una línea vacía). Nunca lanza una excepción.
    """
    if not linea.startswith(PREFIJO_EVENTO):
        return None
    partes = linea[len(PREFIJO_EVENTO):].split(':')
    nombre = partes[0]
    if not nombre or not all(c.isupper() or c.isdigit() or c == '_' for c in nombre):
        return None
    return Event(name=nombre, args=tuple(partes[1:]))
