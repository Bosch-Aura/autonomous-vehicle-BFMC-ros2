"""Adaptador de YOLO (Ultralytics) al contrato de detector de la lógica.

Ultralytics y PyTorch son pesados y no vienen con ROS: se importan recién al crear
el detector. Así el resto del paquete, y sus tests, funcionan sin instalarlos.
En la Jetson se instalan con `pip install ultralytics` (PyTorch lo trae JetPack).
"""
import os
from typing import Sequence

import numpy as np

from bfmc_sign_detection.sign_detection_logic import RawDetection

AYUDA_DE_INSTALACION = (
    'Instalá Ultralytics con "pip install ultralytics" (en la Jetson, PyTorch lo trae JetPack).')


class YoloDetector:
    """Detecta señales con un modelo YOLO de Ultralytics."""

    def __init__(self, model_path: str, device: str, confidence: float):
        ruta = os.path.expanduser(model_path)
        if not os.path.isfile(ruta):
            raise FileNotFoundError(
                'no existe el modelo de señales: %s. Descargalo con "descargar_modelo".' % ruta)
        try:
            from ultralytics import YOLO
        except ImportError as error:
            raise ImportError(
                'no se pudo importar Ultralytics. ' + AYUDA_DE_INSTALACION) from error

        self._modelo = YOLO(ruta)
        self._device = device or None
        self._confidence = confidence

    def detect(self, bgr: np.ndarray) -> Sequence[RawDetection]:
        resultados = self._modelo.predict(
            bgr, conf=self._confidence, device=self._device, verbose=False)
        detecciones = []
        for resultado in resultados:
            nombres = resultado.names
            cajas = resultado.boxes
            coordenadas = cajas.xyxy.tolist()
            confianzas = cajas.conf.tolist()
            clases = cajas.cls.tolist()
            for (x_min, y_min, x_max, y_max), confianza, clase in zip(
                    coordenadas, confianzas, clases):
                detecciones.append(RawDetection(
                    class_name=str(nombres[int(clase)]), confidence=float(confianza),
                    x_min=x_min, y_min=y_min, x_max=x_max, y_max=y_max))
        return detecciones


def crear_detector_yolo(model_path: str, device: str, confidence: float) -> YoloDetector:
    """Fábrica por defecto del nodo. Los tests inyectan otra con un detector falso."""
    return YoloDetector(model_path, device, confidence)
