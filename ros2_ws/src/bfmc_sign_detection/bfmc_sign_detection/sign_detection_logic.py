"""Lógica pura de bfmc_sign_detection: de las detecciones de un modelo a señales con distancia.

No importa ROS ni ningún framework de detección. El detector (por ejemplo YOLO) es
intercambiable: la lógica solo necesita un objeto con un método `detect(bgr)` que
devuelva detecciones crudas. Así se prueba con un detector falso, sin el modelo.

Qué hace con cada detección:

1. Descarta las de confianza baja.
2. Traduce la clase del modelo al tipo de señal del sistema (`class_map`). Una clase
   que no está en el mapa se descarta.
3. Estima la distancia con la profundidad ALINEADA al color: la mediana de los
   valores válidos en una ventana centrada en la caja. La mediana resiste a los
   píxeles con profundidad inválida o del fondo.
"""
from dataclasses import dataclass
import math
from typing import Mapping, Optional, Protocol, Sequence

import numpy as np


@dataclass(frozen=True)
class RawDetection:
    """Una detección tal como la entrega el modelo, con la caja en píxeles."""

    class_name: str
    confidence: float
    x_min: float
    y_min: float
    x_max: float
    y_max: float


class Detector(Protocol):
    """Lo que la lógica necesita de un detector."""

    def detect(self, bgr: np.ndarray) -> Sequence[RawDetection]:
        """Devuelve las detecciones de una imagen BGR."""


@dataclass(frozen=True)
class SignResult:
    """Una señal lista para publicar."""

    label: str
    confidence: float
    distance_m: float
    x_min: int
    y_min: int
    x_max: int
    y_max: int


@dataclass(frozen=True)
class SignConfig:
    """Parámetros de la lógica. Sus valores vienen de config/params.yaml."""

    class_map: Mapping[str, str]
    min_confidence: float
    depth_window_fraction: float
    min_depth_m: float
    max_depth_m: float
    min_depth_pixels: int

    def __post_init__(self):
        if not self.class_map:
            raise ValueError('class_map no puede estar vacío')
        if not 0.0 <= self.min_confidence <= 1.0:
            raise ValueError('min_confidence debe estar en [0, 1]')
        if not 0.0 < self.depth_window_fraction <= 1.0:
            raise ValueError('depth_window_fraction debe estar en (0, 1]')
        if not 0.0 <= self.min_depth_m < self.max_depth_m:
            raise ValueError('el rango de profundidad debe cumplir 0 <= mínimo < máximo')
        if self.min_depth_pixels < 1:
            raise ValueError('min_depth_pixels debe ser al menos 1')


def parse_class_map(entradas: Sequence[str]) -> dict:
    """Convierte ['clase:etiqueta', ...] en un diccionario.

    Lanza ValueError si alguna entrada es inválida.
    """
    mapa = {}
    for entrada in entradas:
        clase, separador, etiqueta = entrada.partition(':')
        if not separador or not clase.strip() or not etiqueta.strip():
            raise ValueError('entrada inválida en class_map: "%s". Formato: clase:etiqueta'
                             % entrada)
        if clase.strip() in mapa:
            raise ValueError('la clase "%s" está repetida en class_map' % clase.strip())
        mapa[clase.strip()] = etiqueta.strip()
    return mapa


class SignProcessor:
    """Combina un detector y la profundidad para producir las señales de un cuadro."""

    def __init__(self, detector: Detector, config: SignConfig):
        self._detector = detector
        self._config = config

    def process(self, bgr: np.ndarray, depth_m: Optional[np.ndarray]) -> list:
        """Devuelve las señales del cuadro, de mayor a menor confianza.

        Args:
            bgr: imagen de color.
            depth_m: profundidad alineada al color, en metros, o None si no hay. Puede
                tener otra resolución que el color: las cajas se escalan.
        """
        cfg = self._config
        alto, ancho = bgr.shape[:2]

        senales = []
        for deteccion in self._detector.detect(bgr):
            etiqueta = cfg.class_map.get(deteccion.class_name)
            if etiqueta is None or deteccion.confidence < cfg.min_confidence:
                continue
            caja = self._caja(deteccion, ancho, alto)
            if caja is None:
                continue
            senales.append(SignResult(
                label=etiqueta,
                confidence=float(deteccion.confidence),
                distance_m=self._distancia(caja, depth_m, ancho, alto),
                x_min=caja[0], y_min=caja[1], x_max=caja[2], y_max=caja[3]))

        senales.sort(key=lambda s: s.confidence, reverse=True)
        return senales

    @staticmethod
    def _caja(deteccion, ancho, alto):
        """Recorta la caja a la imagen. Devuelve None si queda sin área."""
        x_min = max(0, int(math.floor(deteccion.x_min)))
        y_min = max(0, int(math.floor(deteccion.y_min)))
        x_max = min(ancho, int(math.ceil(deteccion.x_max)))
        y_max = min(alto, int(math.ceil(deteccion.y_max)))
        if x_max <= x_min or y_max <= y_min:
            return None
        return x_min, y_min, x_max, y_max

    def _distancia(self, caja, depth_m, ancho, alto):
        """Mediana de la profundidad válida en una ventana centrada en la caja, o NaN."""
        if depth_m is None:
            return math.nan
        cfg = self._config
        alto_d, ancho_d = depth_m.shape[:2]
        escala_x, escala_y = ancho_d / ancho, alto_d / alto

        x_min, y_min, x_max, y_max = caja
        centro_x, centro_y = (x_min + x_max) / 2.0, (y_min + y_max) / 2.0
        medio_ancho = (x_max - x_min) * cfg.depth_window_fraction / 2.0
        medio_alto = (y_max - y_min) * cfg.depth_window_fraction / 2.0

        izquierda = max(0, int(math.floor((centro_x - medio_ancho) * escala_x)))
        derecha = min(ancho_d, int(math.ceil((centro_x + medio_ancho) * escala_x)))
        arriba = max(0, int(math.floor((centro_y - medio_alto) * escala_y)))
        abajo = min(alto_d, int(math.ceil((centro_y + medio_alto) * escala_y)))
        if derecha <= izquierda or abajo <= arriba:
            return math.nan

        ventana = depth_m[arriba:abajo, izquierda:derecha]
        validos = ventana[np.isfinite(ventana)
                          & (ventana >= cfg.min_depth_m) & (ventana <= cfg.max_depth_m)]
        if validos.size < cfg.min_depth_pixels:
            return math.nan
        return float(np.median(validos))
