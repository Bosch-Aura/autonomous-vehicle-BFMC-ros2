"""Detector `l_adaptativo`: canal L con umbral adaptativo + piso fijo (multi umbral).

Motivación (informe, 4.3.1, 4.3.3 y 5.2.4.4): la misma marca vial da intensidades distintas
según la iluminación, y la pista tiene sombras y sol directo en lugares que cambian. Un
umbral fijo que anda con sol pierde la línea en la sombra, y al revés.

Segmentación:
1. Canal L (luminosidad) de HLS: separa el brillo del color.
2. Opcional, CLAHE (`clahe_clip` > 0): ecualiza el contraste por zonas.
3. Umbral adaptativo: un píxel es blanco si es `l_offset` más claro que el promedio de su
   zona de `l_block_size_px` de lado. Sigue a la línea aunque la zona entera esté en sombra.
4. Piso fijo `l_min`: en una zona uniforme y oscura el umbral adaptativo también marca
   ruido; lo que es más oscuro que el piso no es línea, se vea como se vea.

El resultado es la intersección de 3 y 4 (los "múltiples umbrales" del informe). El resto
del pipeline es el común.
"""
import cv2
import numpy as np

from bfmc_lane_detection.lane_detection_logic import LaneDetector


class LAdaptativo(LaneDetector):
    """Pipeline común con segmentación por canal L adaptativa."""

    def _segmentar(self, bgr):
        cfg = self.config
        luminosidad = cv2.cvtColor(bgr, cv2.COLOR_BGR2HLS)[:, :, 1]
        if cfg.clahe_clip > 0.0:
            clahe = cv2.createCLAHE(clipLimit=cfg.clahe_clip, tileGridSize=(8, 8))
            luminosidad = clahe.apply(luminosidad)
        adaptativo = cv2.adaptiveThreshold(
            luminosidad, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY,
            cfg.l_block_size_px, -float(cfg.l_offset))
        piso = np.where(luminosidad >= cfg.l_min, 255, 0).astype(np.uint8)
        return cv2.bitwise_and(adaptativo, piso)
