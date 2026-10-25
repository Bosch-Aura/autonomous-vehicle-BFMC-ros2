"""Conversión de mensajes sensor_msgs/Image a arreglos de numpy.

Se hace a mano, sin cv_bridge: evita una dependencia y los problemas de
compatibilidad entre cv_bridge y la versión de numpy.
"""
import numpy as np

CANALES_DE_COLOR = {'bgr8': 3, 'rgb8': 3}
PROFUNDIDAD = {'16UC1': 'u2', '32FC1': 'f4'}


def _filas(alto, ancho, bytes_por_pixel, paso, datos):
    if paso < ancho * bytes_por_pixel:
        raise ValueError('el paso (%d) es menor que el ancho de la fila (%d)'
                         % (paso, ancho * bytes_por_pixel))
    if len(datos) < paso * alto:
        raise ValueError('los datos (%d bytes) no alcanzan para %d filas de %d bytes'
                         % (len(datos), alto, paso))
    crudo = np.frombuffer(bytes(datos[:paso * alto]), dtype=np.uint8).reshape(alto, paso)
    return crudo[:, :ancho * bytes_por_pixel]


def imagen_a_bgr(alto, ancho, codificacion, paso, datos) -> np.ndarray:
    """Devuelve la imagen de color como alto x ancho x 3 de uint8 en orden BGR.

    Soporta `bgr8` y `rgb8`. Lanza ValueError si la codificación no está soportada
    o el tamaño de los datos no coincide.
    """
    if codificacion not in CANALES_DE_COLOR:
        raise ValueError('codificación de color no soportada: "%s". Soportadas: %s'
                         % (codificacion, ', '.join(sorted(CANALES_DE_COLOR))))
    canales = CANALES_DE_COLOR[codificacion]
    imagen = _filas(alto, ancho, canales, paso, datos).reshape(alto, ancho, canales)
    if codificacion == 'rgb8':
        imagen = imagen[:, :, ::-1]
    return np.ascontiguousarray(imagen)


def profundidad_a_metros(alto, ancho, codificacion, paso, es_bigendian, datos,
                         metros_por_unidad) -> np.ndarray:
    """Devuelve la profundidad como alto x ancho de float32, en metros.

    Soporta `16UC1` (enteros; se multiplican por `metros_por_unidad`, 0,001 si el
    valor está en milímetros) y `32FC1` (ya en metros). Un 0 en `16UC1` significa
    "sin medición" y se convierte en NaN. Lanza ValueError si la codificación no
    está soportada o el tamaño de los datos no coincide.
    """
    if codificacion not in PROFUNDIDAD:
        raise ValueError('codificación de profundidad no soportada: "%s". Soportadas: %s'
                         % (codificacion, ', '.join(sorted(PROFUNDIDAD))))
    tipo = ('>' if es_bigendian else '<') + PROFUNDIDAD[codificacion]
    bytes_por_pixel = np.dtype(tipo).itemsize
    crudo = np.ascontiguousarray(_filas(alto, ancho, bytes_por_pixel, paso, datos))
    valores = crudo.view(tipo).reshape(alto, ancho)

    if codificacion == '16UC1':
        metros = valores.astype(np.float32) * np.float32(metros_por_unidad)
        metros[valores == 0] = np.nan
        return metros
    return valores.astype(np.float32)
