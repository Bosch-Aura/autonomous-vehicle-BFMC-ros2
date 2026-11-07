"""Conversión de un mensaje sensor_msgs/Image a un arreglo BGR de numpy.

Se hace a mano, sin cv_bridge: evita una dependencia y los problemas de
compatibilidad entre cv_bridge y la versión de numpy.
"""
import numpy as np

CANALES = {'bgr8': 3, 'rgb8': 3}


def imagen_a_bgr(alto: int, ancho: int, codificacion: str, paso: int, datos) -> np.ndarray:
    """Devuelve la imagen como alto x ancho x 3 de uint8 en orden BGR.

    Soporta `bgr8` y `rgb8`. Respeta `paso` (bytes por fila), que puede incluir
    relleno. Lanza ValueError si la codificación no está soportada o el tamaño de
    los datos no coincide.
    """
    if codificacion not in CANALES:
        raise ValueError('codificación no soportada: "%s". Soportadas: %s'
                         % (codificacion, ', '.join(sorted(CANALES))))
    canales = CANALES[codificacion]
    if paso < ancho * canales:
        raise ValueError('el paso (%d) es menor que el ancho de la fila (%d)'
                         % (paso, ancho * canales))
    if len(datos) < paso * alto:
        raise ValueError('los datos (%d bytes) no alcanzan para %d filas de %d bytes'
                         % (len(datos), alto, paso))

    filas = np.frombuffer(bytes(datos[:paso * alto]), dtype=np.uint8).reshape(alto, paso)
    imagen = filas[:, :ancho * canales].reshape(alto, ancho, canales)
    if codificacion == 'rgb8':
        imagen = imagen[:, :, ::-1]
    return np.ascontiguousarray(imagen)


# Profundidad (REP 118): 16UC1 en milímetros, 32FC1 en metros. 0 es "sin dato".
PROFUNDIDAD = {'16UC1': (np.dtype('<u2'), 0.001), '32FC1': (np.dtype('<f4'), 1.0)}


def profundidad_a_metros(alto: int, ancho: int, codificacion: str, paso: int, datos,
                         big_endian: bool = False) -> np.ndarray:
    """Devuelve la profundidad como alto x ancho de float64 en metros (0 o NaN = sin dato).

    Soporta `16UC1` (milímetros) y `32FC1` (metros). Lanza ValueError si la codificación
    no está soportada o el tamaño de los datos no coincide.
    """
    if codificacion not in PROFUNDIDAD:
        raise ValueError('codificación de profundidad no soportada: "%s". Soportadas: %s'
                         % (codificacion, ', '.join(sorted(PROFUNDIDAD))))
    tipo, escala = PROFUNDIDAD[codificacion]
    if big_endian:
        tipo = tipo.newbyteorder('>')
    if paso < ancho * tipo.itemsize:
        raise ValueError('el paso (%d) es menor que el ancho de la fila (%d)'
                         % (paso, ancho * tipo.itemsize))
    if len(datos) < paso * alto:
        raise ValueError('los datos (%d bytes) no alcanzan para %d filas de %d bytes'
                         % (len(datos), alto, paso))
    filas = np.frombuffer(bytes(datos[:paso * alto]), dtype=np.uint8).reshape(alto, paso)
    valores = filas[:, :ancho * tipo.itemsize].copy().view(tipo).reshape(alto, ancho)
    return valores.astype(np.float64) * escala
