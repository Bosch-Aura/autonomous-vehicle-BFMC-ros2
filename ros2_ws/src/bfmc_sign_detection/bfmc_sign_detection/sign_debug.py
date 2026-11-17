"""Imagen de depuración de las señales: la cámara con una caja por cada señal detectada."""
import math

import cv2

COLOR_CAJA_BGR = (0, 200, 0)
COLOR_TEXTO_BGR = (0, 0, 0)
GROSOR_CAJA_PX = 2
ESCALA_TEXTO = 0.5


def texto_de_senal(senal):
    """«stop 0.87 · 1.2 m». Sin distancia (NaN), solo la etiqueta y la confianza."""
    texto = '%s %.2f' % (senal.label, senal.confidence)
    if not math.isnan(senal.distance_m):
        texto += ' | %.1f m' % senal.distance_m
    return texto


def dibujar_senales(bgr, senales):
    """Devuelve una copia de `bgr` con la caja y el texto de cada señal. No modifica la entrada."""
    salida = bgr.copy()
    for senal in senales:
        esquina_a = (int(senal.x_min), int(senal.y_min))
        esquina_b = (int(senal.x_max), int(senal.y_max))
        cv2.rectangle(salida, esquina_a, esquina_b, COLOR_CAJA_BGR, GROSOR_CAJA_PX)
        texto = texto_de_senal(senal)
        (ancho, alto), base = cv2.getTextSize(
            texto, cv2.FONT_HERSHEY_SIMPLEX, ESCALA_TEXTO, 1)
        # El rótulo va arriba de la caja; si no entra, adentro.
        arriba = max(esquina_a[1] - alto - base, 0)
        cv2.rectangle(salida, (esquina_a[0], arriba),
                      (esquina_a[0] + ancho, arriba + alto + base), COLOR_CAJA_BGR, -1)
        cv2.putText(salida, texto, (esquina_a[0], arriba + alto),
                    cv2.FONT_HERSHEY_SIMPLEX, ESCALA_TEXTO, COLOR_TEXTO_BGR, 1, cv2.LINE_AA)
    return salida
