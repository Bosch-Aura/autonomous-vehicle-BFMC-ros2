r"""Protocolo serial Jetson <-> ESP32 (ADR-0001). Sin ROS y sin puerto serial.

Formato: un objeto JSON por línea, terminada en '\n'.

    Jetson -> ESP32   {"t":"drive","seq":12,"v":250,"s":-30}   v en mm/s, s en décimas de grado
                      {"t":"brake","seq":13}
    ESP32 -> Jetson   {"t":"ultra","id":"front","dist_cm":87,"valid":true}
                      {"t":"status","fw":"0.1.0","wd":false,"last_seq":12}
"""
from dataclasses import dataclass
import json

# El firmware lee la secuencia con hasta 9 dígitos.
SEQ_MAX = 999_999_999


@dataclass(frozen=True)
class UltraMessage:
    """Lectura del ultrasónico delantero."""

    dist_cm: int
    valid: bool


@dataclass(frozen=True)
class StatusMessage:
    """Estado que reporta el ESP32."""

    firmware: str
    watchdog_tripped: bool
    last_seq: int


def encode_drive(seq: int, v_mm_s: int, s_decimas: int) -> bytes:
    """Arma la línea de un comando de conducción."""
    _validar_seq(seq)
    return ('{"t":"drive","seq":%d,"v":%d,"s":%d}\n' % (seq, v_mm_s, s_decimas)).encode('ascii')


def encode_brake(seq: int) -> bytes:
    """Arma la línea de un comando de freno."""
    _validar_seq(seq)
    return ('{"t":"brake","seq":%d}\n' % seq).encode('ascii')


def _validar_seq(seq):
    if not 0 <= seq <= SEQ_MAX:
        raise ValueError('seq fuera de rango (0..%d): %s' % (SEQ_MAX, seq))


def _es_entero(valor):
    return isinstance(valor, int) and not isinstance(valor, bool)


def parse_line(linea: str):
    """Interpreta una línea del ESP32.

    Devuelve un `UltraMessage`, un `StatusMessage` o None si la línea no es un
    mensaje válido. Una línea inválida nunca lanza una excepción: se descarta.
    """
    try:
        datos = json.loads(linea)
    except ValueError:
        return None
    if not isinstance(datos, dict):
        return None

    tipo = datos.get('t')
    if tipo == 'ultra':
        dist = datos.get('dist_cm')
        valida = datos.get('valid')
        if datos.get('id') != 'front' or not _es_entero(dist) or dist < 0:
            return None
        if not isinstance(valida, bool):
            return None
        return UltraMessage(dist_cm=dist, valid=valida)

    if tipo == 'status':
        firmware = datos.get('fw')
        watchdog = datos.get('wd')
        ultima_seq = datos.get('last_seq')
        if not isinstance(firmware, str) or not isinstance(watchdog, bool):
            return None
        if not _es_entero(ultima_seq) or ultima_seq < 0:
            return None
        return StatusMessage(firmware=firmware, watchdog_tripped=watchdog, last_seq=ultima_seq)

    return None
