"""Conversión de unidades SI de ROS al formato del ESP32.

Es el único lugar del sistema donde se convierten las unidades: dentro de ROS
todo es SI (m/s, rad) y el ESP32 espera un duty de tracción (0-255), un sentido
de marcha aparte y una posición de servo (izquierda < centro < derecha).
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Esp32Limits:
    """Escalas del ESP32. Tienen que coincidir con Brain-Aura/hardware.env."""

    max_speed_mps: float
    max_steer_rad: float
    duty_start: int
    duty_max: int
    servo_left: int
    servo_center: int
    servo_right: int

    def __post_init__(self):
        if self.max_speed_mps <= 0.0 or self.max_steer_rad <= 0.0:
            raise ValueError('max_speed_mps y max_steer_rad deben ser positivos')
        if not 0 < self.duty_start < self.duty_max <= 255:
            raise ValueError('debe cumplirse 0 < duty_start < duty_max <= 255')
        if not self.servo_left < self.servo_center < self.servo_right:
            raise ValueError('debe cumplirse servo_left < servo_center < servo_right')


def _finito(valor):
    return valor if math.isfinite(valor) else 0.0


def to_duty(speed_mps: float, limits: Esp32Limits):
    """Convierte una velocidad en m/s a (sentido, duty).

    El sentido es True adelante, False atrás, o None si la velocidad es 0 (no hay
    que cambiarlo). Cualquier velocidad distinta de 0 da un duty de al menos
    `duty_start`: por debajo de ese umbral el motor no arranca desde parado y el
    firmware lo ignora en silencio. Un valor no finito se trata como 0.
    """
    speed_mps = _finito(speed_mps)
    if speed_mps == 0.0:
        return None, 0
    fraccion = min(1.0, abs(speed_mps) / limits.max_speed_mps)
    duty = limits.duty_start + fraccion * (limits.duty_max - limits.duty_start)
    return speed_mps > 0.0, int(round(duty))


def to_servo(steering_rad: float, limits: Esp32Limits) -> int:
    """Convierte un ángulo de dirección en rad (positivo a la izquierda, REP-103) al servo.

    `max_steer_rad` corresponde al tope del servo en cada lado; un ángulo mayor se
    recorta. Un valor no finito se trata como 0 (recto).
    """
    fraccion = max(-1.0, min(1.0, _finito(steering_rad) / limits.max_steer_rad))
    if fraccion >= 0.0:
        servo = limits.servo_center - fraccion * (limits.servo_center - limits.servo_left)
    else:
        servo = limits.servo_center - fraccion * (limits.servo_right - limits.servo_center)
    return int(round(servo))
