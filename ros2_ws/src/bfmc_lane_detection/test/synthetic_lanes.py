"""Imágenes sintéticas de carriles para los tests: posición conocida, sin necesitar la pista."""
import cv2
import numpy as np

BLANCO = (255, 255, 255)


def lienzo(ancho, alto):
    return np.zeros((alto, ancho, 3), dtype=np.uint8)


def linea(imagen, desde, hasta, grosor=6, color=BLANCO):
    cv2.line(imagen, desde, hasta, color, grosor)
    return imagen


def carril_cenital(ancho, alto, x_izquierda, x_derecha, desvio_arriba=0, grosor=8):
    """Vista cenital con dos líneas. `desvio_arriba` corre el extremo superior de ambas en
    píxeles hacia la derecha: un valor positivo es un carril que sigue hacia la derecha."""
    imagen = lienzo(ancho, alto)
    for x in (x_izquierda, x_derecha):
        if x is not None:
            linea(imagen, (x, alto - 1), (x + desvio_arriba, 0), grosor)
    return imagen


def a_vista_de_camara(cenital, matriz, ancho, alto):
    """Proyecta una vista cenital a la imagen de cámara, con la inversa de `matriz`.

    `matriz` lleva de la cámara a la vista cenital; la imagen resultante es la que
    vería la cámara si el piso fuera esa vista cenital.
    """
    return cv2.warpPerspective(cenital, np.linalg.inv(matriz), (ancho, alto))
