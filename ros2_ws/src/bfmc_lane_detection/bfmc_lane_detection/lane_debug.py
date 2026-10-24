"""Imágenes de depuración de bfmc_lane_detection: muestran qué ve el detector.

No importa ROS. Recibe el `Analisis` que devuelve `LaneDetector.analizar` y dibuja dos vistas:

- La de la cámara: la imagen real con lo que el detector toma por blanco pintado de rojo,
  el trapecio de `warp_src` (amarillo), el límite de la región de interés (cian), las
  curvas encontradas (verde la izquierda, azul la derecha) y el error lateral (magenta).
- La cenital: la vista desde arriba con la máscara de blancos, las ventanas deslizantes,
  las curvas, la posición del auto (línea amarilla) y la fila donde se mide (naranja).

Los textos van sin tildes: las fuentes de OpenCV no las dibujan.
"""
from bfmc_lane_detection.lane_detection_logic import Analisis, LaneDetector
import cv2
import numpy as np

ROJO = (0, 0, 255)
VERDE = (0, 200, 0)
AZUL = (255, 140, 0)
AMARILLO = (0, 255, 255)
CIAN = (255, 255, 0)
MAGENTA = (255, 0, 255)
NARANJA = (0, 165, 255)
BLANCO = (255, 255, 255)
GRIS_CLARO = (200, 200, 200)
GRIS_OSCURO = (90, 90, 90)
NEGRO = (0, 0, 0)

PASO_CURVA_PX = 8
OPACIDAD_BLANCOS = 0.6
LIMITE_PX = 10000
FUENTE = cv2.FONT_HERSHEY_SIMPLEX


def renderizar(detector: LaneDetector, bgr: np.ndarray):
    """Analiza la imagen y devuelve (vista de cámara, vista cenital), ambas BGR."""
    analisis = detector.analizar(bgr)
    return dibujar_camara(detector, bgr, analisis), dibujar_cenital(detector, analisis)


def dibujar_camara(detector: LaneDetector, bgr: np.ndarray, analisis: Analisis) -> np.ndarray:
    """Dibuja lo que vio el detector sobre la imagen de la cámara. No modifica `bgr`."""
    cfg = detector.config
    alto, ancho = bgr.shape[:2]

    capa = bgr.copy()
    capa[analisis.mascara > 0] = ROJO
    imagen = cv2.addWeighted(capa, OPACIDAD_BLANCOS, bgr, 1.0 - OPACIDAD_BLANCOS, 0.0)

    y_roi = int(alto * cfg.roi_top_fraction)
    cv2.line(imagen, (0, y_roi), (ancho - 1, y_roi), CIAN, 1)

    esquinas = np.array(cfg.warp_src, dtype=np.float64).reshape(4, 2) * [ancho, alto]
    cv2.polylines(imagen, [esquinas.astype(np.int32).reshape(-1, 1, 2)], True, AMARILLO, 2)

    inversa = np.linalg.inv(detector.warp_matrix(ancho, alto))
    for ajuste, color in ((analisis.izquierda, VERDE), (analisis.derecha, AZUL)):
        if ajuste is not None:
            _polilinea(imagen, _curva(ajuste[0], cfg.warp_height), inversa, color, 2)
    centro = analisis.centro
    if centro is not None:
        auto = (cfg.warp_width / 2.0, centro['y_px'])
        carril = (centro['centro_px'], centro['y_px'])
        _polilinea(imagen, np.array([auto, carril], dtype=np.float64), inversa, MAGENTA, 3)

    _escribir(imagen, _resumen(analisis), 8, 8)
    _escribir(imagen, ['rojo=blanco  amarillo=trapecio  cian=region  verde/azul=lineas'],
              8, alto - 24)
    return imagen


def dibujar_cenital(detector: LaneDetector, analisis: Analisis) -> np.ndarray:
    """Dibuja la vista desde arriba con las ventanas, las curvas y la medición."""
    cfg = detector.config
    alto, ancho = analisis.cenital.shape
    imagen = np.zeros((alto, ancho, 3), dtype=np.uint8)
    imagen[analisis.cenital > 0] = GRIS_CLARO

    resultado = analisis.resultado
    y_banda = int(alto * cfg.intersection_band_top_fraction)
    color_banda = MAGENTA if resultado.intersection_detected else GRIS_OSCURO
    cv2.line(imagen, (0, y_banda), (ancho - 1, y_banda), color_banda, 1)

    for lado, color in (('izq', VERDE), ('der', AZUL)):
        traza = analisis.trazas.get(lado)
        if not traza:
            continue
        for x_ini, y_ini, x_fin, y_fin, pixeles in traza['ventanas']:
            grosor = 2 if pixeles >= cfg.min_pixels else 1
            cv2.rectangle(imagen, (x_ini, y_ini), (x_fin - 1, y_fin - 1), color, grosor)
        if traza['base'] is not None:
            cv2.circle(imagen, (int(traza['base']), alto - 1), 6, color, -1)

    for ajuste, color in ((analisis.izquierda, VERDE), (analisis.derecha, AZUL)):
        if ajuste is not None:
            puntos = np.clip(_curva(ajuste[0], alto), -LIMITE_PX, LIMITE_PX).astype(np.int32)
            cv2.polylines(imagen, [puntos.reshape(-1, 1, 2)], False, color, 3)

    cv2.line(imagen, (ancho // 2, 0), (ancho // 2, alto - 1), AMARILLO, 1)
    centro = analisis.centro
    if centro is not None:
        # La fila de referencia puede caer justo en el borde de abajo (y = alto): se la mete.
        y = min(int(round(centro['y_px'])), alto - 1)
        cv2.line(imagen, (0, y), (ancho - 1, y), NARANJA, 1)
        cv2.circle(imagen, (int(round(centro['centro_px'])), y), 6, MAGENTA, -1)

    _escribir(imagen, _resumen(analisis), 8, 8)
    _escribir(imagen, ['amarillo=auto  naranja=fila de medicion'], 8, alto - 24)
    return imagen


def _curva(coeficientes, alto):
    """Puntos (x, y) de la curva x(y) = a*y^2 + b*y + c para y de 0 a `alto`."""
    ys = np.arange(0, alto, PASO_CURVA_PX, dtype=np.float64)
    return np.column_stack([np.polyval(coeficientes, ys), ys])


def _polilinea(imagen, puntos_cenital, inversa, color, grosor):
    """Dibuja sobre la imagen de cámara unos puntos dados en la vista cenital."""
    puntos = np.asarray(puntos_cenital, dtype=np.float64).reshape(-1, 1, 2)
    en_camara = cv2.perspectiveTransform(puntos, inversa).reshape(-1, 2)
    en_camara = en_camara[np.isfinite(en_camara).all(axis=1)]
    if len(en_camara) < 2:
        return
    en_camara = np.clip(en_camara, -LIMITE_PX, LIMITE_PX).astype(np.int32)
    cv2.polylines(imagen, [en_camara.reshape(-1, 1, 2)], False, color, grosor)


def _resumen(analisis):
    resultado = analisis.resultado
    if resultado.detected:
        primera = 'carril: SI (%s)' % analisis.centro['modo']
        segunda = 'desvio %+.3f m  rumbo %+.3f rad  conf %.2f' % (
            resultado.lateral_error_m, resultado.heading_error_rad, resultado.confidence)
    else:
        primera = 'carril: NO detectado'
        segunda = 'sin lineas validas'
    lineas = [primera, segunda]
    if resultado.intersection_detected:
        lineas.append('INTERSECCION')
    return lineas


def _escribir(imagen, lineas, x, y):
    """Escribe líneas de texto con un borde negro para que se lean sobre cualquier fondo."""
    for i, linea in enumerate(lineas):
        posicion = (x, y + 14 + 18 * i)
        cv2.putText(imagen, linea, posicion, FUENTE, 0.5, NEGRO, 3, cv2.LINE_AA)
        cv2.putText(imagen, linea, posicion, FUENTE, 0.5, BLANCO, 1, cv2.LINE_AA)
