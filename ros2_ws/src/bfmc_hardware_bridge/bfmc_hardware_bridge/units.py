"""Conversión de unidades SI de ROS al formato del ESP32.

Es el único lugar del sistema donde se convierten las unidades: dentro de ROS
todo es SI (m/s, rad) y el ESP32 espera mm/s y décimas de grado.
"""
import math


def _limitar(valor, maximo):
    return max(-maximo, min(maximo, valor))


def to_esp32_units(speed_mps: float, steering_rad: float, max_speed_mps: float,
                   max_steer_rad: float):
    """Convierte velocidad y dirección a (mm/s, décimas de grado), enteros y recortados.

    Un valor que no es un número finito (NaN o infinito) se trata como 0: ante la
    duda, el auto no se mueve ni gira.

    Args:
        speed_mps: velocidad deseada en m/s. Positivo avanza.
        steering_rad: ángulo de dirección en rad. Positivo a la izquierda.
        max_speed_mps: velocidad máxima permitida (el signo se recorta en ambos sentidos).
        max_steer_rad: ángulo máximo permitido (se recorta en ambos sentidos).
    """
    if not math.isfinite(speed_mps):
        speed_mps = 0.0
    if not math.isfinite(steering_rad):
        steering_rad = 0.0

    v_max = round(max_speed_mps * 1000.0)
    s_max = round(math.degrees(max_steer_rad) * 10.0)

    v_mm_s = _limitar(round(speed_mps * 1000.0), v_max)
    s_decimas = _limitar(round(math.degrees(steering_rad) * 10.0), s_max)
    return int(v_mm_s), int(s_decimas)
