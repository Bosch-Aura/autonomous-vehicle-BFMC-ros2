"""Profundidad sintética de un piso visto por una cámara de altura e inclinación conocidas.

Cámara: x a la derecha, y hacia abajo, z hacia adelante. Una cámara inclinada `pitch`
grados hacia abajo (y `roll` grados con el lado derecho más alto) a `altura` metros del
piso. Se usa para probar la calibración sin la cámara ni la pista.
"""
import math

import numpy as np

# Intrínsecos parecidos a los del color de la D435 a 640 x 480.
FX = FY = 615.0
CX, CY = 320.0, 240.0
ANCHO, ALTO = 640, 480


def normal_del_piso(pitch_deg, roll_deg=0.0):
    """Devuelve la normal del piso (hacia arriba) en coordenadas de la cámara."""
    p, r = math.radians(pitch_deg), math.radians(roll_deg)
    # Sin roll, "arriba" es -y inclinado hacia atrás por el pitch: (0, -cos p, -sin p).
    # El roll gira esa dirección alrededor del eje óptico.
    arriba = np.array([0.0, -math.cos(p), -math.sin(p)])
    giro = np.array([[math.cos(r), math.sin(r), 0.0],
                     [-math.sin(r), math.cos(r), 0.0],
                     [0.0, 0.0, 1.0]])
    return giro.T @ arriba


def profundidad_del_piso(altura_m, pitch_deg, roll_deg=0.0, ruido_m=0.0,
                         fraccion_basura=0.0, semilla=0):
    """Imagen de profundidad en metros (0 donde no se ve piso).

    `ruido_m` suma ruido gaussiano. `fraccion_basura` reemplaza esa fracción de la mitad de
    abajo por un "obstáculo": un bloque de puntos mucho más cerca que el piso.
    """
    normal = normal_del_piso(pitch_deg, roll_deg)
    v, u = np.mgrid[0:ALTO, 0:ANCHO].astype(np.float64)
    rayos = np.stack(((u - CX) / FX, (v - CY) / FY, np.ones_like(u)), axis=-1)
    coseno = rayos @ normal
    with np.errstate(divide='ignore'):
        z = np.where(coseno < -1e-9, -altura_m / coseno, 0.0)
    azar = np.random.default_rng(semilla)
    if ruido_m > 0.0:
        z = np.where(z > 0.0, z + azar.normal(0.0, ruido_m, z.shape), 0.0)
    if fraccion_basura > 0.0:
        # Un bloque en la mitad de abajo, de lado a lado, que ocupa esa fracción.
        filas = int(round(fraccion_basura * ALTO / 2))
        z[ALTO - filas:, :] = 0.0
        z[ALTO - filas:, :] = azar.uniform(0.32, 0.4, (filas, ANCHO))
    return z


def pared(distancia_m):
    """Algo plano tapando toda la cámara, perpendicular al eje óptico."""
    return np.full((ALTO, ANCHO), distancia_m)


def a_16uc1(metros):
    """Profundidad en milímetros como la publica la RealSense (0 = sin dato)."""
    return np.clip(np.round(metros * 1000.0), 0, 65535).astype('<u2')
