"""Detector `hsv_ventanas`: umbral HSV fijo + ventanas deslizantes (el de siempre, A2-137).

La segmentación es la de LaneDetector: blanco = brillo alto (V >= white_hsv_min[2]) y poca
saturación (S <= white_hsv_max[1]). Es simple y rápida, pero un umbral fijo es frágil con
sombras y sol directo (informe, 4.3.1 y 4.3.3).
"""
from bfmc_lane_detection.lane_detection_logic import LaneDetector


class HsvVentanas(LaneDetector):
    """El pipeline común con la segmentación HSV de LaneDetector, sin cambios."""
